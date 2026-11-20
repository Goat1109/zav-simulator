#include <epic_planner/corridor_pass.h>
#include <array>
#include <stdexcept>

namespace fast_planner {
namespace {
using Point = CorridorPass::Point;

Point pointAt(const FrontierRoute &path, double arc) {
  for (size_t i = 1; i < path.size(); ++i) {
    const Point edge = path[i] - path[i - 1];
    const double length = edge.norm();
    if (length < 1.e-5) continue;
    if (arc <= length) return path[i - 1] + edge * float(arc / length);
    arc -= length;
  }
  return path.back();
}

Point tangent(const FrontierRoute &path, double arc, double span) {
  if (path.size() < 2) return Point::Zero();
  const Point delta = pointAt(path, arc + span) - pointAt(path, arc);
  if (delta.norm() < .25f) return Point::Zero();
  return delta.normalized();
}
} // namespace

double levelGuideRenewalDistance(double speed, double acceleration, double jerk,
                                 double handoff_time, double clearance) {
  // Distance to brake from measured speed with zero initial/final acceleration.
  // The extra travel covers a planning cycle and delivery of the next guide.
  const double acc = acceleration > 0. ? acceleration : 3.;
  double brake_time = speed / acc;
  if (jerk > 0.) {
    brake_time = speed * jerk <= acc * acc ? 2. * std::sqrt(speed / jerk)
                                           : speed / acc + acc / jerk;
  }
  return 3. * clearance + speed * (handoff_time + .5 * brake_time);
}

FrontierRoute forwardLevelPrefix(FrontierRoute route, const Point &heading,
                                 double arc_limit, float clearance) {
  if (route.size() < 2 || arc_limit <= 0.) return {};
  splitFrontierRoute(route, arc_limit);
  float progress = 0.f;
  for (size_t i = 1; i < route.size(); ++i) {
    const Point delta = route[i] - route.front();
    const float along = delta.dot(heading);
    const float lateral = (delta - along*heading).head<2>().norm();
    if (along + .25f*clearance < progress || std::abs(delta.z()) > clearance ||
        lateral > 2.f*clearance + .1f*std::max(0.f,along)) {
      route.resize(i);
      break;
    }
    progress = std::max(progress, along);
  }
  return route;
}

FrontierRoute roundObservedTurns(const FrontierRoute &path, float radius, float spacing,
    const std::function<bool(const Point &, const Point &)> &clear) {
  if (path.size() < 3 || radius <= 0.f || spacing <= 0.f) return path;
  FrontierRoute result{path.front()};
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    const Point incoming = path[i] - path[i-1], outgoing = path[i+1] - path[i];
    const float first = incoming.norm(), second = outgoing.norm();
    float trim = std::min({radius, .48f*first, .48f*second});
    if (trim >= spacing) {
      const Point u = incoming / first, v = outgoing / second;
      const float alignment = u.dot(v);
      bool rounded = false;
      // Preserve genuine inspection reversals. Smooth only transit geometry
      // and only over segments individually witnessed in the live scan map.
      while (alignment > -.8f && alignment < .985f && trim >= spacing) {
        const Point start = path[i] - trim*u, end = path[i] + trim*v;
        FrontierRoute curve{start};
        const int samples = std::max(2, int(std::ceil(2.f*trim / spacing)));
        bool safe = clear(result.back(), start);
        for (int j = 1; safe && j <= samples; ++j) {
          const float t = float(j)/samples, s = 1.f-t;
          const Point next = s*s*start + 2.f*s*t*path[i] + t*t*end;
          safe = clear(curve.back(), next);
          curve.push_back(next);
        }
        if (safe) {
          result.insert(result.end(), curve.begin(), curve.end());
          rounded = true;
          break;
        }
        trim *= .5f;
      }
      if (rounded) continue;
    }
    result.push_back(path[i]);
  }
  result.push_back(path.back());
  return result;
}

FrontierRoute trimDepartureRetrace(const FrontierRoute &path,const Point &velocity,float clearance,
    const std::function<bool(const Point &,const Point &)> &clear) {
  if(path.size()<3 || !velocity.allFinite() || velocity.norm()<.7f || clearance<=0.f) return {};
  const Point origin=path.front(),axis=velocity.normalized();
  const Point initial=path[1]-origin;
  if(initial.norm()<.015f || initial.dot(axis)>-.28f*initial.norm()) return {};
  float minimum=0.f,furthest=0.f,arc=0.f;
  bool reentered=false;
  FrontierRoute winner;
  for(size_t i=1;i<path.size() && arc<7.f*clearance;++i) {
    const Point edge=path[i]-path[i-1];
    const int steps=std::max(1,int(std::ceil(edge.norm()/(.55f*clearance))));
    for(int j=1;j<=steps;++j) {
      if(arc+edge.norm()*float(j)/steps>7.f*clearance) return winner;
      const Point join=path[i-1]+float(j)/steps*edge;
      const Point chord=join-origin;
      const float along=chord.dot(axis),length=chord.norm();
      minimum=std::min(minimum,along);
      // A true return into the previous lane or a long side detour owns
      // its observation. Only trim a short loop around this same departure.
      if(minimum< -1.4f*clearance || (!reentered && (chord-along*axis).norm()>1.6f*clearance)) return winner;
      if(reentered && along<furthest-.45f*clearance) return winner;
      if(along>.4f*clearance) reentered=true;
      furthest=std::max(furthest,along);
      if(!reentered || minimum>-.27f*clearance || length<1.2f*clearance || along<.5f*length) continue;
      Point outgoing=edge;
      if(j==steps && i+1<path.size()) outgoing=path[i+1]-join;
      if(outgoing.norm()<.015f || chord.dot(outgoing)<.38f*length*outgoing.norm()) continue;
      if(!clear(origin,join)) continue;
      FrontierRoute connected{origin,join};
      const size_t tail=j==steps ? i+1 : i;
      connected.insert(connected.end(),path.begin()+tail,path.end());
      winner=std::move(connected);
    }
    arc+=edge.norm();
  }
  return winner;
}

FrontierRoute alignMeasuredDeparture(const FrontierRoute &path, const Point &velocity, float clearance,
    float spacing, const std::function<bool(const Point &, const Point &)> &clear) {
  if (path.size()<3 || !velocity.allFinite() || velocity.norm()<.45f || clearance<=0.f || spacing<=0.f)
    return path;
  const Point heading=velocity.normalized();
  const Point origin=path.front();
  const Point first=path[1]-origin;
  if (first.norm()<.001f || first.normalized().dot(heading)>.91f) return path;

  // A closest-point reconnect can add a small sideways or backward leg in
  // front of an otherwise forward guide. Preserve real return legs and turns.
  FrontierRoute prefix=path;
  const float horizon=std::clamp(1.75f*velocity.norm()+clearance,2.5f*clearance,5.f*clearance);
  auto suffix=splitFrontierRoute(prefix,horizon);
  if (prefix.size()<3) return path;
  float last=0.f, highest=origin.z(), lowest=origin.z();
  for (size_t i=1;i<prefix.size();++i) {
    const Point delta=prefix[i]-origin;
    const float along=delta.dot(heading);
    if (along<last-.55f*clearance) return path;
    last=std::max(last,along);
    highest=std::max(highest,prefix[i].z());
    lowest=std::min(lowest,prefix[i].z());
  }
  const Point chord=prefix.back()-origin;
  const float distance=chord.norm();
  if (distance<1.4f*clearance || chord.dot(heading)<.30f*distance) return path;
  const Point terminal=(prefix.back()-prefix[prefix.size()-2]).normalized();
  if (terminal.dot(chord)<.15f*distance) return path;
  const int count=std::max(3,int(std::ceil(1.4f*distance/spacing)));
  for(float fraction:{.38f,.22f}) {
    const Point a=origin+fraction*distance*heading, b=prefix.back()-fraction*distance*terminal;
    FrontierRoute rounded{origin};
    bool valid=true;
    for(int j=1;j<=count && valid;++j) {
      const float t=float(j)/count,s=1.f-t;
      const Point next=s*s*s*origin+3.f*s*s*t*a+3.f*s*t*t*b+t*t*t*prefix.back();
      float separation=std::numeric_limits<float>::infinity();
      for(size_t i=1;i<prefix.size();++i) {
        const Point edge=prefix[i]-prefix[i-1];
        const float ratio=std::clamp((next-prefix[i-1]).dot(edge)/std::max(.00001f,edge.squaredNorm()),0.f,1.f);
        separation=std::min(separation,(next-prefix[i-1]-ratio*edge).norm());
      }
      valid=separation<=.9f*clearance && next.z()>=lowest-.05f*clearance &&
          next.z()<=highest+.05f*clearance && clear(rounded.back(),next);
      rounded.push_back(next);
    }
    if(valid) {
      if(!suffix.empty()) rounded.insert(rounded.end(),suffix.begin()+1,suffix.end());
      return rounded;
    }
  }
  return path;
}

FrontierRoute improveObservedPreview(const FrontierRoute &route,const Point &velocity,float clearance,
    float resolution,const std::function<bool(const Point &,const Point &)> &clear) {
  if(route.size()<3 || velocity.norm()<.55f || clearance<=0.f || resolution<=0.f) return {};
  const float desired=std::clamp(1.4f*velocity.norm()+clearance,2.5f*clearance,7.f*clearance);
  const Point origin=route.front(),heading=velocity.normalized();
  float arc=0.f,old_preview=0.f,best=0.f;
  bool limited=false;
  size_t chosen=0;
  for(size_t j=1;j<route.size();++j) {
    arc+=(route[j]-route[j-1]).norm();
    if(arc>desired+3.f*clearance) break;
    const Point chord=route[j]-origin;
    const float length=chord.norm();
    if(length<.01f) continue;
    const Point axis=chord/length;
    float deviation=0.f,progress=0.f;
    bool ordered=true;
    for(size_t k=1;k<j;++k) {
      const Point to=route[k]-origin;
      const float along=to.dot(axis);
      if(along<progress-.25f*clearance || along>length+.25f*clearance) {ordered=false;break;}
      progress=std::max(progress,along);
      deviation=std::max(deviation,(to-std::clamp(along,0.f,length)*axis).norm());
    }
    if(!ordered) break;
    if(!limited && deviation<=.75f*resolution) {
      old_preview=arc;
      if(old_preview>desired) return {};
      continue;
    }
    limited=true;
    if(deviation>1.2f*clearance || axis.dot(heading)<.45f || length<1.8f*clearance ||
       length<old_preview+1.1f*clearance || length<1.4f*old_preview || length<=best) continue;
    if(!clear(origin,route[j])) continue;
    best=length;chosen=j;
  }
  if(!chosen) return {};
  FrontierRoute updated{origin};
  updated.insert(updated.end(),route.begin()+chosen,route.end());
  return updated;
}

float movingSightDistance(const FrontierRoute &route,float spacing,float tolerance,float horizon) {
  if(route.size()<2 || spacing<=0.f || tolerance<=0.f || horizon<=0.f) return 0.f;
  const Point origin=route.front();
  FrontierRoute seen;
  float length=0.f,best=0.f;
  for(size_t i=1;i<route.size() && length<horizon;++i) {
    const Point d=route[i]-route[i-1];
    const float segment=d.norm();
    if(segment<.01f) continue;
    const int steps=std::max(1,int(std::ceil(segment/spacing)));
    for(int j=1;j<=steps;++j) {
      const float at=std::min(horizon,length+segment*j/steps);
      const Point p=route[i-1]+((at-length)/segment)*d,v=p-origin;
      const float norm=std::max(.00001f,v.squaredNorm());
      for(const Point &previous:seen) {
        const Point relative=previous-origin;
        const float fraction=std::clamp(relative.dot(v)/norm,0.f,1.f);
        if((relative-fraction*v).norm()>tolerance) return best;
      }
      seen.push_back(p);best=at;
      if(at>=horizon-.001f)return best;
    }
    length+=segment;
  }
  return best;
}

FrontierRoute centerObservedAisle(const Point &velocity,const FrontierRoute &route,float clearance,
    const std::function<float(const Point &)> &room,const std::function<bool(const Point &,const Point &)> &clear) {
  if(route.size()<2 || clearance<=0.f || velocity.head<2>().norm()<1.5f) return {};
  float length=0.f;
  for(size_t i=1;i<route.size();++i) length+=(route[i]-route[i-1]).norm();
  const float reach=std::min(length,std::clamp(3.2f*velocity.head<2>().norm()+4.f*clearance,
                                               10.f*clearance,23.f*clearance));
  if(reach<8.f*clearance) return {};
  const Point body=route.front(),end=pointAt(route,reach),delta=end-body;
  const float along=delta.head<2>().norm();
  if(along<.96f*reach || std::abs(delta.z())>1.3f*clearance) return {};
  const Point axis=Point(delta.x(),delta.y(),0.f)/along,lateral(-axis.y(),axis.x(),0.f);
  if(velocity.dot(axis)<.89f*velocity.norm()) return {};
  const int samples=std::max(10,int(std::ceil(reach/(.7f*clearance))));
  FrontierRoute baseline;
  float previous=0.f;
  for(int i=0;i<=samples;++i) {
    const Point v=pointAt(route,reach*i/samples),d=v-body;
    if(d.dot(axis)<previous-.12f*clearance || std::abs(d.dot(lateral))>.95f*clearance ||
       std::abs(v.z()-body.z())>1.45f*clearance) return {};
    previous=d.dot(axis);baseline.push_back(v);
  }
  // Score clearance over the measured corridor as a whole. One wide
  // doorway must not pull an otherwise straight lane against the wall.
  auto reserve=[&](const FrontierRoute &points) {
    std::vector<float> distances;
    for(size_t i=1;i+1<points.size();++i) {
      const float distance=room(points[i]);
      if(distance<clearance+.012f) return -1.f;
      distances.push_back(std::min(2.15f*clearance,distance));
    }
    std::sort(distances.begin(),distances.end());
    float mean=0.f;for(float d:distances)mean+=d;
    return .46f*distances[distances.size()/6]+.54f*mean/distances.size();
  };
  const float original=reserve(baseline);
  if(original>1.75f*clearance) return {};
  const float first=.56f*reach,last=.78f*reach;
  const Point at_first=body+(first/reach)*delta,at_last=body+(last/reach)*delta;
  const auto shape=[&](const Point &offset,float arc)->Point {
    // The first straight is long enough for rolling target handoff. Two
    // short downstream joins return to the same surveyed route and goal.
    if(arc<=first) return body+arc/first*(at_first+offset-body);
    if(arc<=last) return at_first+offset+(arc-first)/(last-first)*(at_last-at_first);
    return at_last+offset+(arc-last)/(reach-last)*(end-at_last-offset);
  };
  struct Option{float score;Point offset;};std::vector<Option> candidates;
  for(float side:{0.f,.45f,-.45f,.9f,-.9f,1.2f,-1.2f})for(float z:{0.f,.36f,-.36f,.7f,-.7f}) {
    const Point offset=clearance*(side*lateral+z*Point::UnitZ());
    FrontierRoute points;
    for(int i=0;i<=samples;++i) points.push_back(shape(offset,reach*i/samples));
    const float score=reserve(points)-.14f*offset.norm();
    if(score>original+.13f*clearance) candidates.push_back({score,offset});
  }
  std::stable_sort(candidates.begin(),candidates.end(),[](const auto &a,const auto &b){return a.score>b.score;});
  for(const auto &candidate:candidates) {
    FrontierRoute selected{body,at_first+candidate.offset,at_last+candidate.offset,end};
    // Do not put a short viewing target in front of a fast vehicle just
    // to gain clearance. Require directional continuity at both joins.
    const Point a=(selected[1]-body).normalized(),b=(selected[2]-selected[1]).normalized();
    const Point e=(end-selected[2]).normalized();
    if(a.dot(b)<.93f || b.dot(e)<.86f) continue;
    bool free=true;
    for(size_t i=1;i<selected.size() && free;++i) free=clear(selected[i-1],selected[i]);
    if(!free) continue;
    auto prefix=route;
    auto tail=splitFrontierRoute(prefix,reach);
    if(tail.size()>1) {
      // A section of the previous lane may now be obstructed. A new
      // clear near-field path is usable only with a verified onward tail.
      for(size_t i=1;i<tail.size() && free;++i) free=clear(tail[i-1],tail[i]);
      if(!free) continue;
      selected.insert(selected.end(),tail.begin()+1,tail.end());
    }
    return selected;
  }
  return {};
}

// Approximate how far a chord-limited follower can see as its origin moves
// into a bend. At the physical end of the guide there is no next segment.
static float transitVisibility(const FrontierRoute &path,float clearance,float resolution) {
  const float horizon=4.5f*clearance;
  float total=0.f;
  for(size_t i=1;i<path.size();++i) total+=(path[i]-path[i-1]).norm();
  float sum=0.f,lowest=horizon,weight=0.f;
  for(float start=0.f;start<std::min(4.f*clearance,total-1.4f*clearance);start+=.65f*clearance) {
    auto head=path;
    const auto tail=start==0.f ? path : splitFrontierRoute(head,start);
    const float available=std::min(horizon,total-start);
    const float sight=movingSightDistance(tail,1.5f*resolution,.75f*resolution,available);
    const float limit=std::min(horizon,sight*horizon/available);
    const float w=1.f/(1.f+.5f*start/clearance);
    lowest=std::min(lowest,limit);
    sum+=w*limit;weight+=w;
  }
  return weight>0.f ? .55f*lowest+.45f*sum/weight : 0.f;
}

FrontierRoute shapeTransitApproach(const FrontierRoute &path,const Point &velocity,float clearance,float resolution,
    const std::function<float(const Point &)> &room, const std::function<bool(const Point &,const Point &)> &clear) {
  if(path.size()<3 || clearance<=0.f || resolution<=0.f || !velocity.allFinite() || velocity.norm()<.6f) return {};
  float length=0.f;
  for(size_t i=1;i<path.size();++i) length+=(path[i]-path[i-1]).norm();
  const float window=std::min(length,9.f*clearance);
  if(window<2.6f*clearance) return {};
  const Point origin=path.front(),entry=velocity.normalized();
  const bool rising=pointAt(path,window).z()>=origin.z()+.48f*clearance;
  auto reserve=[&](const FrontierRoute &route) {
    std::array<float,9> samples;
    const float reach=std::min(window,5.f*clearance);
    for(size_t j=0;j<samples.size();++j)
      samples[j]=std::clamp(room(pointAt(route,reach*(j+1)/samples.size())),0.f,clearance+.55f);
    std::sort(samples.begin(),samples.end());
    return .5f*(samples[1]+samples[2]);
  };
  auto measured=path;
  splitFrontierRoute(measured,std::min(length,window+3.f*clearance));
  const float old_sight=transitVisibility(measured,clearance,resolution);
  const float old_room=reserve(measured);
  if(old_sight>3.8f*clearance && old_room>clearance+.18f) return {};
  float best_gain=.28f*clearance;
  FrontierRoute best;

  // Rejoin the ordered, measured guide partway through the turn. This can
  // start banking before the outgoing stair flight is entirely in view.
  for(float span=window;span>2.5f*clearance;span-=1.6f*clearance) {
    const Point end=pointAt(path,span);
    Point exit=pointAt(path,std::min(length,span+.75f*clearance))-pointAt(path,span-.75f*clearance);
    if(exit.norm()<.2f*clearance) continue;
    exit.normalize();
    const float alignment=entry.dot(exit);
    if(alignment>.94f || (!rising && (alignment<-.38f || (end-origin).norm()<.68f*span))) continue;
    if(entry.dot(exit)<-.60f && (end-origin-(end-origin).dot(entry)*entry).norm()<.85f*clearance) continue;
    auto head=path;auto rest=splitFrontierRoute(head,span);
    auto local_rest=rest;
    if(local_rest.size()>1) splitFrontierRoute(local_rest,window+3.f*clearance-span);
    const int samples=std::max(6,int(std::ceil(span/(1.65f*resolution))));
    Point side(exit.x()-entry.x(),exit.y()-entry.y(),0.f);
    if(side.norm()>.1f) side.normalize();
    else if(entry.head<2>().norm()>.1f) side=Point(-entry.y(),entry.x(),0.f).normalized();
    else side=Point::UnitX();
    const std::array<Point,7> offsets{Point::Zero(),.55f*clearance*side,-.55f*clearance*side,
        .46f*clearance*Point::UnitZ(),-.46f*clearance*Point::UnitZ(),
        .42f*clearance*(side+Point::UnitZ()),.42f*clearance*(-side+Point::UnitZ())};
    for(float lead:{0.f,.7f*clearance,1.35f*clearance}) {
      const Point start=origin+lead*entry;
      if(lead>.33f*span || (lead>0.f && !clear(origin,start))) continue;
      for(float scale:{.30f,.44f,.56f}) for(const Point &offset:offsets) {
        const Point first=start+scale*(span-lead)*entry;
        const Point last=end-scale*(span-lead)*exit;
        FrontierRoute curve{origin};
        if(lead>0.f) curve.push_back(start);
        bool valid=true;
        for(int j=1;j<=samples;++j) {
          const float t=float(j)/samples,u=1.f-t;
          const Point at=u*u*u*start+3.f*u*u*t*first+3.f*u*t*t*last+t*t*t*end+16.f*t*t*u*u*offset;
          const Point reference=pointAt(path,lead+(span-lead)*t);
          if((at-reference).norm()>1.65f*clearance || !clear(curve.back(),at)) {valid=false;break;}
          curve.push_back(at);
        }
        if(!valid) continue;
        auto local=curve;
        if(local_rest.size()>1) local.insert(local.end(),local_rest.begin()+1,local_rest.end());
        const float new_room=reserve(local);
        if(new_room<old_room-.075f) continue;
        const float sight_gain=transitVisibility(local,clearance,resolution)-old_sight;
        if(sight_gain<-.12f*clearance) continue;
        const float gain=sight_gain+1.6f*(new_room-old_room);
        if(gain<=best_gain) continue;
        if(rest.size()>1) curve.insert(curve.end(),rest.begin()+1,rest.end());
        best_gain=gain;best=std::move(curve);
      }
    }
  }
  return best;
}

FrontierRoute smoothLevelApproach(const FrontierRoute &path,const Point &velocity,float clearance,
    float resolution,const std::function<bool(const Point &,const Point &)> &clear) {
  if(path.size()<3 || clearance<=0.f || resolution<=0.f || velocity.head<2>().norm()<.65f) return {};
  const Point origin=path.front();
  float length=0.f;
  for(size_t i=1;i<path.size();++i) length+=(path[i]-path[i-1]).norm();
  const float horizon=std::min(length,std::clamp(3.f*velocity.head<2>().norm()+3.f*clearance,
                                                7.5f*clearance,23.f*clearance));
  if(horizon<6.f*clearance) return {};
  const float tolerance=.75f*resolution;
  auto error=[](const Point &a,const Point &p,const Point &b) {
    const Point line=b-a;
    const float t=std::clamp((p-a).dot(line)/std::max(.00001f,line.squaredNorm()),0.f,1.f);
    return (p-(a+t*line)).norm();
  };
  auto preview=[&](const FrontierRoute &route) {
    float arc=0.f;
    for(size_t k=1;k<route.size();++k) {
      bool straight=true;
      for(size_t j=1;j<k && straight;++j) straight=error(route.front(),route[j],route[k])<=tolerance;
      if(!straight) break;
      arc+=(route[k]-route[k-1]).norm();
    }
    return arc;
  };
  FrontierRoute winner;float best=0.f;
  for(float fraction:{1.f,.78f,.58f}) {
    const float span=fraction*horizon;
    if(span<5.5f*clearance) continue;
    const int count=std::max(6,int(std::ceil(span/(.7f*clearance))));
    const Point end=pointAt(path,span);
    const float horizontal=(end-origin).head<2>().norm();
    if(horizontal<.94f*span || std::abs(end.z()-origin.z())>1.45f*clearance) continue;
    const Point axis((end.x()-origin.x())/horizontal,(end.y()-origin.y())/horizontal,0.f);
    if(axis.dot(velocity)<.87f*velocity.head<2>().norm()) continue;
    FrontierRoute reference{origin};
    float previous=0.f,z_min=end.z(),z_max=end.z();
    bool level=true;
    for(int i=1;i<=count && level;++i) {
      const Point p=pointAt(path,span*i/count),delta=p-origin;
      const float along=delta.dot(axis);
      if(along<previous-.1f*clearance || (delta-along*axis).head<2>().norm()>1.35f*clearance)
        level=false;
      if(2*i>=count) {z_min=std::min(z_min,p.z());z_max=std::max(z_max,p.z());}
      reference.push_back(p);previous=along;
    }
    if(!level || z_max-z_min>.85f*clearance) continue;
    const float baseline=preview(reference);
    if(baseline>=.86f*span) continue;
    Point exit=pointAt(path,std::min(length,span+.8f*clearance))-
               pointAt(path,std::max(0.f,span-.8f*clearance));
    if(exit.dot(axis)<.1f || exit.normalized().dot(axis)<.87f) continue;
    exit/=exit.dot(axis);
    const Point measured=velocity/velocity.dot(axis),chord=(end-origin)/horizontal;
    // Interpolate the transverse and vertical slopes independently of arc
    // length. The plane of each cross section is fixed, so a candidate can
    // never retreat to a previous aisle or cut a branch at a junction.
    for(float blend:{0.f,.42f,.8f}) {
      Point incoming=chord+blend*(measured-chord),outgoing=chord+blend*(exit-chord);
      incoming.z()=std::clamp(incoming.z(),-.24f,.24f);
      outgoing.z()=std::clamp(outgoing.z(),-.24f,.24f);
      FrontierRoute curve{origin};
      for(int i=1;i<=count;++i) {
        const float t=float(i)/count,u=1.f-t;
        const Point p=u*u*(1.f+2.f*t)*origin+t*t*(3.f-2.f*t)*end+
                      horizontal*(u*u*t*incoming-u*t*t*outgoing);
        curve.push_back(p);
      }
      const float reach=preview(curve);
      const float gain=reach-baseline;
      if(gain<=std::max(1.3f*clearance,.28f*baseline) || gain<=best) continue;
      bool safe=true;
      for(size_t i=1;i<curve.size() && safe;++i) safe=clear(curve[i-1],curve[i]);
      if(!safe) continue;
      auto prefix=path;
      auto tail=splitFrontierRoute(prefix,span);
      if(tail.size()>1) curve.insert(curve.end(),tail.begin()+1,tail.end());
      best=gain;winner=std::move(curve);
    }
  }
  return winner;
}

FrontierRoute rollingVisibilityGuide(const Point &velocity,const FrontierRoute &path,float clearance,
    float resolution,const std::function<float(const Point &)> &room,
    const std::function<bool(const Point &,const Point &)> &clear,float required_preview) {
  if(path.size()<3 || clearance<=0.f || resolution<=0.f || !velocity.allFinite()) return {};
  struct Sample {Point p; float arc; size_t next;};
  const Point origin=path.front();
  const float speed=velocity.norm(),spacing=2.f*resolution;
  float horizon=std::clamp(2.25f*speed+1.6f*clearance,3.2f*clearance,8.f*clearance);
  bool straight_aisle=false;
  if(required_preview>horizon && speed>clearance) {
    const float wanted=std::min(required_preview,24.f*clearance);
    const Point forward=pointAt(path,wanted)-origin;
    // A fast level aisle needs enough measured sight distance to cover
    // braking and handoff. Keep the local window for a steep switchback.
    if(forward.head<2>().norm()>.83f*wanted && std::abs(forward.z())<1.5f*clearance &&
       velocity.head<2>().dot(forward.head<2>())>.82f*velocity.head<2>().norm()*forward.head<2>().norm())
      {horizon=wanted;straight_aisle=true;}
  }
  std::vector<Sample> samples{{origin,0.f,1}};
  float since=0.f,arc=0.f;
  for(size_t i=1;i<path.size() && arc<horizon+3.f*clearance;++i) {
    const Point edge=path[i]-path[i-1];const float length=edge.norm();
    if(length<.001f) continue;
    for(float step=spacing-since;step<length-.001f;step+=spacing)
      samples.push_back({path[i-1]+(step/length)*edge,arc+step,i});
    since=std::fmod(since+length,spacing);arc+=length;
    samples.push_back({path[i],arc,i+1});
  }
  if(samples.size()<4) return {};
  const Point heading=speed>.35f ? velocity/speed : (samples[1].p-origin).normalized();
  const float tolerance=.75f*resolution;
  float preview=0.f;
  std::vector<size_t> eligible;
  for(size_t j=1;j<samples.size() && samples[j].arc<horizon+spacing;++j) {
    const Point delta=samples[j].p-origin;const float length=delta.norm();
    if(length<.02f) continue;
    const Point axis=delta/length;
    float progression=0.f,deviation=0.f;bool ordered=true;
    for(size_t i=1;i<j;++i) {
      const Point to=samples[i].p-origin;const float along=to.dot(axis);
      if(along<progression-.20f*clearance || along>length+.20f*clearance) {ordered=false;break;}
      progression=std::max(progression,along);
      deviation=std::max(deviation,(to-std::clamp(along,0.f,length)*axis).norm());
    }
    if(!ordered || length<.68f*samples[j].arc || deviation>1.55f*clearance) break;
    if(eligible.empty() && deviation<=tolerance && preview>=samples[j-1].arc-.001f) {
      preview=samples[j].arc;
      if(preview>=.85f*horizon) return {};
    }
    if(length>=std::max({1.65f*clearance,preview+.65f*clearance,1.3f*preview}) &&
       axis.dot(heading)>.32f) eligible.push_back(j);
  }
  struct Option {size_t j,k; Point aim; float score;};
  std::vector<Option> options;
  for(const size_t j:eligible) {
    const Point center=samples[j].p;
    const Point forward=(center-origin).normalized();
    const Point lateral=forward.head<2>().norm()>.1f ? Point(-forward.y(),forward.x(),0).normalized() : Point::UnitX();
    const Point up=forward.cross(lateral).normalized();
    size_t k=j;
    while(k+1<samples.size() && samples[k].arc<samples[j].arc+1.6f*clearance) ++k;
    const bool final=samples[j].next>=path.size();
    for(const auto &uv:std::array<Eigen::Vector2f,15>{{{0,0},{.45f,0},{-.45f,0},{0,.45f},{0,-.45f},
                     {.8f,0},{-.8f,0},{0,.8f},{0,-.8f},{.42f,.42f},{-.42f,.42f},{.42f,-.42f},{-.42f,-.42f},
                     {1.15f,0},{-1.15f,0}}}) {
      const bool centered=uv.squaredNorm()<.001f;
      if(!centered && (k==j || final)) continue;
      const Point aim=center+clearance*(uv.x()*lateral+uv.y()*up),delta=aim-origin;
      const float length=delta.norm();const Point axis=delta/length;
      const float alignment=axis.dot(heading);
      if(alignment<.38f) continue;
      const size_t join=centered ? j : k;
      Point afterwards=join!=j ? samples[join].p-aim :
          (j+1<samples.size() ? samples[j+1].p-aim : delta);
      if(afterwards.norm()<.015f) afterwards=delta;
      const float exit=axis.dot(afterwards.normalized());
      if(!centered && exit<.05f) continue;
      const float score=length+.8f*clearance*alignment+.8f*clearance*exit-
                        .34f*clearance*uv.norm();
      options.push_back({j,join,aim,score});
    }
  }
  std::stable_sort(options.begin(),options.end(),[](const Option &a,const Option &b){return a.score>b.score;});
  for(const auto &option:options) {
    const bool centered=option.j==option.k;
    if(room(option.aim)<clearance+.012f || !clear(origin,option.aim)) continue;
    size_t join_index=option.k;
    if(!centered && straight_aisle) {
      // Preserve a straight departure across the tight section. Immediately
      // forcing the offset back onto a distant old centreline can point the
      // downstream velocity towards the opposite wall. Join later only if
      // the entire local stretch is still the same observed straight aisle.
      for(float reach:{8.f*clearance,4.f*clearance}) {
        size_t k=option.k;
        while(k+1<samples.size() && samples[k].arc<samples[option.j].arc+reach) ++k;
        if(k<=option.k) continue;
        const Point connection=samples[k].p-option.aim;
        if(connection.norm()<.5f*clearance ||
           connection.normalized().dot((option.aim-origin).normalized())<.86f) continue;
        bool same_aisle=true;
        for(size_t m=option.j;m<=k && same_aisle;++m) {
          const Point delta=samples[m].p-option.aim;
          const float along=std::clamp(delta.dot(connection)/connection.squaredNorm(),0.f,1.f);
          same_aisle=(delta-along*connection).norm()<1.25f*clearance;
        }
        if(same_aisle && clear(option.aim,samples[k].p)) {join_index=k;break;}
      }
    }
    const auto &join=samples[join_index];
    if(!centered && join_index==option.k && !clear(option.aim,join.p)) continue;
    FrontierRoute updated{origin,option.aim};
    if(!centered && (join.p-updated.back()).norm()>.015f) updated.push_back(join.p);
    for(size_t i=join.next;i<path.size();++i)
      if((path[i]-updated.back()).norm()>.005f) updated.push_back(path[i]);
    return updated;
  }
  return {};
}

FrontierRoute repairObservedBend(const FrontierRoute &path,float margin,float spacing,
    const std::function<float(const Point &)> &room,
    const std::function<bool(const Point &,const Point &)> &clear,float horizon_limit) {
  if(path.size()<2 || margin<=0.f || spacing<=0.f) return {};
  float length=0.f;
  for(size_t i=1;i<path.size();++i) length+=(path[i]-path[i-1]).norm();
  if(length<spacing) return {};
  // The obstruction check covers the immediate six clearance radii. Rejoin
  // the original three-dimensional route just beyond that neighbourhood.
  const float horizon=std::min(length,horizon_limit>0.f ? horizon_limit : 9.f*margin);
  struct Candidate {Point p; float cost; int parent;};
  std::vector<Candidate> candidates{{path.front(),0.f,-1}};
  std::vector<int> previous{0};
  const int stations=std::max(2,int(std::ceil(horizon/spacing)));
  for(int j=1;j<=stations;++j) {
    const float arc=horizon*j/stations;
    const Point centre=pointAt(path,arc);
    const Point tangent=pointAt(path,std::min(length,arc+.5f*spacing))-
                        pointAt(path,std::max(0.f,arc-.5f*spacing));
    if(tangent.norm()<.01f) return {};
    Point side(-tangent.y(),tangent.x(),0.f);
    if(side.squaredNorm()<.0001f) side=Point::UnitX();
    side.normalize();
    const Point up=tangent.normalized().cross(side).normalized();
    std::vector<int> next;
    const int radius=j==stations ? 0 : 2;
    for(int sy=-radius;sy<=radius;++sy) for(int sz=-radius;sz<=radius;++sz) {
      if(sy*sy+sz*sz>4) continue;
      const Point offset=.53f*margin*(float(sy)*side+float(sz)*up), p=centre+offset;
      const float clearance=room(p);
      if(clearance<margin+.012f) continue;
      float best=std::numeric_limits<float>::infinity();int parent=-1;
      for(int index:previous) {
        const auto &a=candidates[index];
        const Point delta=p-a.p;
        const float distance=delta.norm();
        if(distance>.01f && distance<1.85f*spacing) {
          float cost=a.cost+distance+.18f*offset.norm()+
                  .12f*spacing*margin/std::max(.18f*margin,clearance-margin);
          if(a.parent>=0) {
            const Point incoming=a.p-candidates[a.parent].p;
            if(incoming.norm()>.01f)
              cost+=.85f*spacing*(1.f-std::clamp(incoming.normalized().dot(delta/distance),-1.f,1.f));
          }
          if(cost<best && clear(a.p,p)) {best=cost;parent=index;}
        }
      }
      if(parent<0) continue;
      next.push_back(int(candidates.size()));
      candidates.push_back({p,best,parent});
    }
    if(next.empty()) return {};
    previous.swap(next);
  }
  FrontierRoute result;
  for(int i=previous.front();i>=0;i=candidates[i].parent) result.push_back(candidates[i].p);
  std::reverse(result.begin(),result.end());
  float arc=0.f;
  for(size_t i=1;i<path.size();++i) {
    arc+=(path[i]-path[i-1]).norm();
    if(arc>horizon+.01f) result.push_back(path[i]);
  }
  // Always preserve the inspection pose, even for a millimetre-long tail.
  result.back()=path.back();
  return result;
}

FrontierRoute fitMeasuredBend(const FrontierRoute &path, float clearance, float spacing,
    const std::function<bool(const Point &, const Point &)> &clear) {
  if (path.size()<3 || spacing<=0.f || clearance<=0.f) return {};
  float length=0.f;
  for (size_t i=1;i<path.size();++i) length+=(path[i]-path[i-1]).norm();
  if (length<3.f*clearance) return {};
  // Measure the bend over metres of travel. A dense, collision-checked
  // staircase polyline can distribute one switchback over twenty vertices.
  // Rounding each vertex independently then leaves a near-stationary pivot.
  const float extent=2.2f*clearance, step=std::min(.65f*clearance,spacing);
  const float limit=std::min(length,16.f*clearance);
  struct Bend { float arc,turn; };
  std::vector<Bend> bends;
  for (float arc=1.2f*clearance;arc<limit-1.2f*clearance;arc+=step) {
    const Point p=pointAt(path,arc);
    const Point in=p-pointAt(path,std::max(0.f,arc-extent));
    const Point out=pointAt(path,std::min(length,arc+extent))-p;
    if (in.norm()<.65f*clearance || out.norm()<.65f*clearance) continue;
    const float turn=1.f-in.normalized().dot(out.normalized());
    if (turn>.36f) bends.push_back({arc,turn});
  }
  std::stable_sort(bends.begin(),bends.end(),[](const Bend &a,const Bend &b){return a.turn>b.turn;});
  std::vector<float> examined;
  for (const auto &bend:bends) {
    if (std::any_of(examined.begin(),examined.end(),[&](float x){return std::abs(bend.arc-x)<extent;})) continue;
    if (examined.size()>=2) break;
    examined.push_back(bend.arc);
    FrontierRoute winner;float best_gain=.12f;
    for (float window:{3.8f*clearance,2.7f*clearance,1.75f*clearance}) {
      const float start=std::max(0.f,bend.arc-window),end=std::min(length,bend.arc+window),span=end-start;
      const int n=std::max(6,int(std::ceil(span/step)));
      const Point a=pointAt(path,start),b=pointAt(path,end);
      Point incoming=pointAt(path,std::min(end,start+.65f*clearance))-pointAt(path,std::max(0.f,start-.65f*clearance));
      Point outgoing=pointAt(path,std::min(length,end+.65f*clearance))-pointAt(path,std::max(start,end-.65f*clearance));
      if (incoming.norm()<.1f || outgoing.norm()<.1f) continue;
      incoming.normalize();outgoing.normalize();
      if(incoming.dot(outgoing)<-.65f) {
        const Point across=b-a-(b-a).dot(incoming)*incoming;
        // A true inspection reversal in a single lane retains its endpoint.
        if(across.norm()<.85f*clearance || b.z()-a.z()<.4f*clearance) continue;
      }
      FrontierRoute original;
      for(int i=0;i<=n;++i) original.push_back(pointAt(path,start+span*i/n));
      auto severity=[&](const FrontierRoute &curve) {
        // Evaluate direction change per metre of travel, including both joins.
        Point last=incoming;float peak=0.f,sum=0.f;
        for(size_t j=1;j<=curve.size();++j) {
          Point next=outgoing;float ds=step;
          if(j<curve.size()) {
            next=curve[j]-curve[j-1];ds=next.norm();
            if(ds<.01f) continue;
            next/=ds;
          }
          const float turn=std::acos(std::clamp(last.dot(next),-1.f,1.f));
          const float k=turn/std::max(.6f*step,ds);
          peak=std::max(peak,k);sum+=turn*k;last=next;
        }
        return clearance*(peak+.18f*sum);
      };
      const float baseline=severity(original);
      if (baseline<.45f) continue;
      Point inward=(outgoing-incoming);
      inward.z()=0.f;
      if(inward.head<2>().norm()>.05f) inward.normalize();
      else inward.setZero();
      const std::array<Point,5> offsets{Point::Zero(),.55f*clearance*inward,-.55f*clearance*inward,
                                        .43f*clearance*Point::UnitZ(),-.43f*clearance*Point::UnitZ()};
      for(const Point &offset:offsets) for(float scale:{.32f,.44f,.54f,.24f}) {
        const Point ca=a+(scale*span)*incoming,cb=b-(scale*span)*outgoing;
        FrontierRoute curve{a};float largest=0.f;
        for (int i=1;i<=n;++i) {
          const float t=float(i)/n,u=1.f-t;
          // The offset and its derivative vanish at both joins. This lets a
          // continuous bend pass outside a handrail or below a low return flight.
          const Point q=u*u*u*a+3.f*u*u*t*ca+3.f*u*t*t*cb+t*t*t*b+
                        (16.f*t*t*u*u)*offset;
          largest=std::max(largest,(q-original[i]).norm());
          curve.push_back(q);
        }
        if (largest>2.f*clearance) continue;
        const float gain=baseline-severity(curve)-.08f*largest/clearance;
        if (gain<best_gain || gain<.16f*baseline) continue;
        bool safe=true;
        for(size_t j=1;j<curve.size() && safe;++j) safe=clear(curve[j-1],curve[j]);
        if (!safe) continue;
        auto head=path;
        auto rest=splitFrontierRoute(head,start);
        if (rest.empty()) continue;
        auto tail=splitFrontierRoute(rest,span);
        head.insert(head.end(),curve.begin()+1,curve.end());
        if (tail.size()>1) head.insert(head.end(),tail.begin()+1,tail.end());
        best_gain=gain;winner=std::move(head);
      }
    }
    if (!winner.empty()) return winner;
  }
  return {};
}

float horizontalObservationGain(const Point &p,float clearance,
    const std::function<bool(const Point &)> &free,const std::function<float(const Point &)> &room) {
  if(clearance<=0.f) return 0.f;
  constexpr size_t count=24;
  std::array<float,count> openings{};
  for(size_t i=0;i<count;++i) {
    const float yaw=2.f*float(M_PI)*i/count;
    const Point direction(std::cos(yaw),std::sin(yaw),0.f);
    for(float height:{-.38f*clearance,0.f,.38f*clearance}) {
      float measured=0.f;
      for(float range=0.f;range<=5.5f*clearance;range+=.4f*clearance) {
        const Point sample=p+range*direction+Point(0,0,height);
        if(room(sample)<.58f*clearance) break;
        if(!free(sample)) {
          // Unknown beyond a witnessed aperture counts as information only.
          // Navigation still ends at the wholly verified measured position.
          if(measured>.5f*clearance) openings[i]+=1.f/3.f;
          break;
        }
        measured=range;
      }
    }
  }
  float useful=0.f;
  for(size_t i=0;i<count;++i)
    useful+=std::min({openings[i],openings[(i+1)%count],openings[(i+count-1)%count]});
  return useful/count;
}

double estimateTransitTime(const FrontierRoute &route,const Point &velocity,float spacing,
    double speed_limit,double acceleration,double jerk) {
  if(route.size()<2 || spacing<=0.f || speed_limit<=0. || acceleration<=0. || jerk<=0.)
    return std::numeric_limits<double>::infinity();
  double length=0.;
  for(size_t i=1;i<route.size();++i) length+=(route[i]-route[i-1]).norm();
  if(length<.01) return std::numeric_limits<double>::infinity();
  const size_t n=std::max<size_t>(2,std::ceil(length/spacing));
  FrontierRoute knots;
  for(size_t i=0;i<=n;++i) knots.push_back(pointAt(route,length*i/n));
  std::vector<double> reachable(n+1,speed_limit),ds(n,0.);
  for(size_t i=0;i<n;++i) ds[i]=(knots[i+1]-knots[i]).norm();
  for(size_t i=1;i<n;++i) {
    const Point a=knots[i]-knots[i-1],b=knots[i+1]-knots[i];
    if(ds[i-1]<.01 || ds[i]<.01) continue;
    const double curvature=std::acos(std::clamp(double(a.dot(b))/(ds[i-1]*ds[i]),-1.,1.))/
                            (.5*(ds[i-1]+ds[i]));
    if(curvature>.001) reachable[i]=std::min({speed_limit,std::sqrt(acceleration/curvature),
                                                       std::cbrt(jerk/(curvature*curvature))});
  }
  if(ds[0]>.01) reachable[0]=std::clamp(double(velocity.dot(knots[1]-knots[0]))/ds[0],0.,speed_limit);
  for(size_t i=1;i<=n;++i)
    reachable[i]=std::min(reachable[i],std::sqrt(reachable[i-1]*reachable[i-1]+2.*acceleration*ds[i-1]));
  for(size_t i=n;i>0;--i)
    reachable[i-1]=std::min(reachable[i-1],std::sqrt(reachable[i]*reachable[i]+2.*acceleration*ds[i-1]));
  double result=0.;
  for(size_t i=0;i<n;++i) result+=2.*ds[i]/std::max(.1,reachable[i]+reachable[i+1]);
  return result;
}

FrontierRoute optimizeTransitWindow(const FrontierRoute &route,const Point &velocity,float clearance,
    double speed_limit,double acceleration,double jerk,const std::function<float(const Point &)> &room,
    const std::function<bool(const Point &,const Point &)> &clear) {
  if(route.size()<3 || clearance<=0.f || speed_limit<=0. || acceleration<=0. || jerk<=0.) return {};
  float length=0.f;
  for(size_t i=1;i<route.size();++i) length+=(route[i]-route[i-1]).norm();
  const float span=std::min(length,14.f*clearance);
  if(span<3.5f*clearance) return {};
  auto original=route;
  auto remainder=splitFrontierRoute(original,span);
  const int count=std::max(4,int(std::ceil(span/(1.28f*clearance))));
  const float step=span/count;
  const Point start=route.front(),end=original.back();
  float peak=0.f;
  for(int i=1;i<count;++i) {
    const float mid=step*i;
    const Point a=pointAt(original,std::max(0.f,mid-1.7f*clearance));
    const Point p=pointAt(original,mid),b=pointAt(original,std::min(span,mid+1.7f*clearance));
    if((p-a).norm()<.2f*clearance || (b-p).norm()<.2f*clearance) continue;
    const Point u=(p-a).normalized(),v=(b-p).normalized();
    const float angle=std::acos(std::clamp(u.dot(v),-1.f,1.f));
    // A real inspection endpoint in a single lane must still be reached.
    const Point across=b-a-(b-a).dot(u)*u;
    if(angle>2.38f && across.norm()<.85f*clearance && std::abs(b.z()-a.z())<.5f*clearance) return {};
    peak=std::max(peak,angle);
  }
  if(peak<.52f) return {};
  struct Choice {Point point;float room,offset;};
  std::vector<std::vector<Choice>> slices(count+1);
  slices.front().push_back({start,room(start),0.f});
  slices.back().push_back({end,room(end),0.f});
  for(int i=1;i<count;++i) {
    const float at=i*step;
    const Point center=pointAt(original,at);
    const Point tangent=pointAt(original,at+.48f*step)-pointAt(original,at-.48f*step);
    if(tangent.norm()<.15f*step) return {};
    Point side(-tangent.y(),tangent.x(),0.f);
    if(side.norm()<.1f) side=Point::UnitX();else side.normalize();
    const Point up=tangent.normalized().cross(side).normalized();
    for(const auto &yz:std::array<Eigen::Vector2f,13>{{{0,0},{.55f,0},{-.55f,0},{1.05f,0},{-1.05f,0},
                   {0,.4f},{0,-.4f},{.55f,.4f},{-.55f,.4f},{.55f,-.4f},{-.55f,-.4f},{0,.8f},{0,-.8f}}}) {
      const Point displacement=clearance*(yz.x()*side+yz.y()*up),p=center+displacement;
      const float available=room(p);
      if(available>clearance+.015f) slices[i].push_back({p,available,displacement.norm()});
    }
    if(slices[i].empty()) return {};
  }
  // Keep an arrival direction with every pair of consecutive cross sections.
  // An equally short path that arrives facing a wall costs more at the next turn.
  const auto bend_time=[&](const Point &incoming,const Point &outgoing) {
    const float a=incoming.norm(),b=outgoing.norm();
    if(a<.02f || b<.02f) return 0.;
    const double angle=std::acos(std::clamp(double(incoming.dot(outgoing))/(a*b),-1.,1.));
    const double curvature=angle/std::max(.15,double(.5f*(a+b)));
    const double limit=curvature<.001 ? speed_limit :
       std::min({speed_limit,std::sqrt(acceleration/curvature),std::cbrt(jerk/(curvature*curvature))});
    return .5*(a+b)*(1./limit-1./speed_limit)+.55*angle*curvature;
  };
  const double original_time=estimateTransitTime(original,velocity,.65f*clearance,speed_limit,acceleration,jerk);
  std::vector<std::vector<std::vector<bool>>> blocked(count+1);
  for(int i=1;i<=count;++i)
    blocked[i].assign(slices[i-1].size(),std::vector<bool>(slices[i].size(),false));
  for(int attempt=0;attempt<6;++attempt) {
    struct Entry {double score=std::numeric_limits<double>::infinity();int predecessor=-1;};
    std::vector<std::vector<std::vector<Entry>>> table(count+1);
    for(int i=1;i<=count;++i)
      table[i].assign(slices[i-1].size(),std::vector<Entry>(slices[i].size()));
    for(int i=1;i<=count;++i) for(size_t j=0;j<slices[i-1].size();++j) for(size_t k=0;k<slices[i].size();++k) {
      if(blocked[i][j][k]) continue;
      const auto &choice=slices[i][k];
      const Point edge=choice.point-slices[i-1][j].point;
      const float distance=edge.norm();
      if(distance<.10f*step || distance>2.25f*step) continue;
      const double cost=distance/speed_limit+.012*choice.offset/clearance+
             .022*step*clearance/std::max(.22f*clearance,choice.room-clearance);
      auto &here=table[i][j][k];
      if(i==1) {
        const Point entering=velocity.norm()>.45f ? (step*velocity.normalized()).eval() : edge;
        here={cost+bend_time(entering,edge),0};continue;
      }
      for(size_t q=0;q<slices[i-2].size();++q) {
        const auto &previous=table[i-1][q][j];
        if(!std::isfinite(previous.score)) continue;
        const Point incoming=slices[i-1][j].point-slices[i-2][q].point;
        const double score=previous.score+cost+bend_time(incoming,edge);
        if(score<here.score) here={score,int(q)};
      }
    }
    int index=-1;double best=std::numeric_limits<double>::infinity();
    for(size_t j=0;j<slices[count-1].size();++j) {
      const double join=remainder.size()<2 ? 0. : bend_time(end-slices[count-1][j].point,remainder[1]-end);
      const double score=table[count][j][0].score+join;
      if(score<best) {best=score;index=int(j);}
    }
    if(index<0) return {};
    std::vector<int> chosen(count+1);chosen[count-1]=index;
    for(int i=count;i>=2;--i) chosen[i-2]=table[i][chosen[i-1]][chosen[i]].predecessor;
    FrontierRoute proposed;
    for(int i=0;i<=count;++i) proposed.push_back(slices[i][chosen[i]].point);
    bool valid=true;
    for(int i=1;i<=count;++i) {
      if(clear(proposed[i-1],proposed[i])) continue;
      blocked[i][chosen[i-1]][chosen[i]]=true;valid=false;
    }
    if(!valid) continue;
    proposed=roundObservedTurns(proposed,1.9f*clearance,.35f*clearance,clear);
    const double predicted=estimateTransitTime(proposed,velocity,.65f*clearance,speed_limit,acceleration,jerk);
    if(predicted>original_time-.09 || predicted>.95*original_time) return {};
    if(remainder.size()>1) proposed.insert(proposed.end(),remainder.begin()+1,remainder.end());
    return proposed;
  }
  return {};
}

FrontierRoute smoothMeasuredBand(const FrontierRoute &path, float clearance, float spacing,
    const std::function<float(const Point &)> &room, const std::function<bool(const Point &,const Point &)> &clear) {
  if(path.size()<3 || clearance<=0.f || spacing<=0.f) return path;
  bool bent=false;
  for(size_t i=1;i+1<path.size();++i) {
    const Point a=path[i]-path[i-1],b=path[i+1]-path[i];
    if(a.norm()>.02f && b.norm()>.02f && a.normalized().dot(b.normalized())<.96f) bent=true;
  }
  if(!bent) return path;
  FrontierRoute guide{path.front()};
  std::vector<bool> fixed{true};
  const float lower=std::min_element(path.begin(),path.end(),[](auto &a,auto &b){return a.z()<b.z();})->z();
  const float upper=std::max_element(path.begin(),path.end(),[](auto &a,auto &b){return a.z()<b.z();})->z();
  for(size_t i=1;i<path.size();++i) {
    const Point edge=path[i]-path[i-1];
    if(edge.norm()<.005f) continue;
    const int count=std::max(1,int(std::ceil(edge.norm()/spacing)));
    const bool reverse=i+1<path.size() && (path[i+1]-path[i]).norm()>.01f &&
        edge.normalized().dot((path[i+1]-path[i]).normalized())<-.75f;
    for(int j=1;j<=count;++j) {
      guide.push_back(path[i-1]+(float(j)/count)*edge);
      fixed.push_back(j==count && (reverse || i+1==path.size()));
    }
  }
  if(guide.size()<3) return path;
  const auto original=guide;
  std::vector<float> available;
  for(const auto &p:guide) available.push_back(room(p));
  const float comfort=clearance+std::min(.45f*clearance,spacing);
  bool changed=false;
  for(int pass=0;pass<8;++pass) {
    float biggest=0.f;
    for(size_t j=1;j+1<guide.size();++j) {
      const size_t i=pass%2 ? guide.size()-1-j : j;
      if(fixed[i]) continue;
      const Point a=guide[i-1],c=guide[i+1],p=guide[i],span=c-a;
      if(span.norm()<.2f*spacing) continue;
      const float f=std::clamp((p-a).dot(span)/span.squaredNorm(),.12f,.88f);
      const Point middle=a+f*span, tangent=span.normalized();
      const float error=(p-middle).norm();
      if(error<.025f*spacing && available[i]>=comfort) continue;
      const Point horizontal=std::abs(tangent.z())<.95f ? Point(-tangent.y(),tangent.x(),0).normalized() : Point::UnitX();
      const Point vertical=tangent.cross(horizontal).normalized();
      const Point blended=p+.8f*(middle-p);
      const float shift=std::min(.34f*spacing,std::max(.08f*spacing,comfort-available[i]));
      const std::array<Point,6> candidates{blended,blended+shift*horizontal,blended-shift*horizontal,
          blended+shift*vertical,blended-shift*vertical,p+.28f*(middle-p)};
      auto cost=[&](const Point &q,float d) {
        const float lack=std::max(0.f,comfort-d);
        return 2.f*(q-middle).squaredNorm()+.045f*(q-original[i]).squaredNorm()+.70f*lack*lack;
      };
      float best=cost(p,available[i]);
      Point selected=p;float selected_room=available[i];
      for(const auto &q:candidates) {
        if((q-original[i]).norm()>1.35f*clearance || std::abs(q.z()-original[i].z())>.80f*clearance ||
           q.z()<lower-.015f || q.z()>upper+.015f) continue;
        const float d=room(q),score=cost(q,d);
        if(score+.0002f*spacing*spacing>=best || d<=clearance || !clear(a,q) || !clear(q,c)) continue;
        selected=q;selected_room=d;best=score;
      }
      if((selected-p).norm()>.001f) {
        guide[i]=selected;available[i]=selected_room;changed=true;
        biggest=std::max(biggest,(selected-p).norm());
      }
    }
    if(biggest<.012f*spacing) break;
  }
  return changed ? guide : path;
}

FrontierRoute observedClimb(const Point &body, const Point &velocity, const FrontierRoute &goals,
    float clearance, float minimum, const std::function<float(const Point &)> &room,
    const std::function<bool(const Point &)> &unvisited) {
  if(goals.empty() || clearance<=0.f || minimum<=0.f || room(body)<=minimum) return {};
  const bool moving=velocity.head<2>().norm()>.55f;
  const float initial_yaw=moving ? std::atan2(velocity.y(),velocity.x()) : 0.f;
  const float initial_slope=moving ? std::clamp(velocity.z()/velocity.head<2>().norm(),-.25f,.9f) : 0.f;
  float maximum_z=body.z();
  for(const auto &g:goals) maximum_z=std::max(maximum_z,g.z());
  auto gap=[&](const Point &p) {
    float closest=std::numeric_limits<float>::infinity();
    for(const auto &g:goals)
      closest=std::min(closest,(g-p).norm()+.48f*(maximum_z-g.z()));
    return closest;
  };
  const float initial_gap=gap(body);
  const bool steep_visible=std::any_of(goals.begin(),goals.end(),[&](const Point &g) {
    const Point d=g-body;
    return d.z()>1.25f*clearance && d.z()>.85f*d.head<2>().norm()+.35f*clearance;
  });
  if(steep_visible) {
    float best=-std::numeric_limits<float>::infinity();
    FrontierRoute direct;
    for(const Point &goal:goals) {
      const Point edge=goal-body;const float length=edge.norm();
      if(edge.z()<=1.25f*clearance || edge.z()<.85f*edge.head<2>().norm() ||
         length<1.3f*clearance || room(goal)<=clearance+.08f || !unvisited(goal)) continue;
      const float alignment=velocity.norm()>.45f ? velocity.normalized().dot(edge/length) : 1.f;
      if(alignment<.55f) continue;
      const int samples=std::max(2,int(std::ceil(length/(.24f*clearance))));
      float available=room(body);
      for(int i=1;i<=samples && available>minimum+.012f;++i)
        available=std::min(available,room(body+(float(i)/samples)*edge));
      if(available<=minimum+.012f) continue;
      const float score=2.f*edge.z()-.25f*length+.8f*clearance*alignment+.4f*std::min(1.6f*clearance,available);
      if(score>best) {direct={body,goal};best=score;}
    }
    if(!direct.empty()) return direct;
  }
  const std::array<float,7> slopes{-.12f,.12f,.40f,.68f,.92f,1.42f,2.3f};
  const float step=1.30f*clearance;
  struct State {Point p;float yaw,slope,turns,margin,rank;int parent;};
  std::vector<State> nodes{{body,initial_yaw,initial_slope,0.f,room(body),0.f,-1}};
  std::vector<int> wave{0};
  int best=-1;
  float best_rank=-std::numeric_limits<float>::infinity();
  for(int depth=0;depth<9 && !wave.empty();++depth) {
    std::vector<State> children;
    for(const int parent:wave) {
      const auto &from=nodes[parent];
      for(int k=0;k<(depth==0 ? 16 : 5);++k) {
        const float delta=(depth==0 ? (k-8)*float(M_PI)/8.f : (k-2)*float(M_PI)/5.f);
        const float yaw=from.yaw+delta;
        for(size_t pitch=0;pitch<(steep_visible ? slopes.size() : size_t(5));++pitch) {
          const float slope=slopes[pitch];
          const Point direction(std::cos(yaw),std::sin(yaw),slope);
          // Newly available steep segments use the same spatial sampling
          // interval as a level step. They still need direct ray evidence all
          // the way through the space above a tread or beside the railing.
          const Point advance=(pitch>=5 ? step/direction.norm() : step)*direction;
          const Point end=from.p+advance;
          if(end.z()<body.z()-.45f*clearance || end.z()>maximum_z+.3f*clearance) continue;
          float available=from.margin;
          for(int j=1;j<=5 && available>minimum+.012f;++j)
            available=std::min(available,room(from.p+(.2f*j)*advance));
          if(available<=minimum+.012f) continue;
          // Cost of turning while moving and changing pitch. A stationary
          // start can take any heading. Keep spatially distinct routes in the
          // beam so a safe switchback survives alongside a shorter dead end.
          float bend=(depth || moving ? std::abs(delta) : 0.f)+.55f*std::abs(slope-from.slope);
          if(steep_visible && (depth || moving)) {
            const Point before(std::cos(from.yaw),std::sin(from.yaw),from.slope);
            bend=std::acos(std::clamp(before.dot(direction)/(before.norm()*direction.norm()),-1.f,1.f));
          }
          const float turns=from.turns+bend;
          const float gain=initial_gap-gap(end),rise=end.z()-body.z();
          const float rank=2.f*gain+1.6f*rise-.22f*(depth+1)*step-.53f*clearance*turns+
              .6f*std::min(available,1.6f*clearance);
          children.push_back({end,yaw,slope,turns,available,rank,parent});
        }
      }
    }
    std::stable_sort(children.begin(),children.end(),[](const State &a,const State &b){return a.rank>b.rank;});
    wave.clear();
    for(const auto &s:children) {
      // A higher-ranked state at nearly the same place may only be a transit
      // point. Test a valid observation before pruning neighbouring states.
      if(depth>=2 && s.rank>best_rank && s.p.z()>body.z()+.9f*clearance &&
          gap(s.p)<initial_gap-.65f*clearance && room(s.p)>clearance+.08f && unvisited(s.p)) {
        best_rank=s.rank;best=int(nodes.size());nodes.push_back(s);
      }
      if(wave.size()>=18) {
        if(s.rank<=best_rank) break;
        continue;
      }
      if(std::any_of(wave.begin(),wave.end(),[&](int j) {
        return (nodes[j].p-s.p).head<2>().norm()<.75f*clearance &&
               std::abs(nodes[j].p.z()-s.p.z())<.55f*clearance &&
               std::cos(nodes[j].yaw-s.yaw)>.4f;
      })) continue;
      wave.push_back(int(nodes.size()));
      nodes.push_back(s);

    }
  }
  FrontierRoute path;
  for(int i=best;i>=0;i=nodes[i].parent) path.push_back(nodes[i].p);
  std::reverse(path.begin(),path.end());
  return path;
}

double observationStepLowerBound(double gap,double radius) {
  if(!std::isfinite(gap) || !std::isfinite(radius) || gap<0. || radius<0.)
    return std::numeric_limits<double>::infinity();
  return gap-.88*std::min(gap,radius);
}

bool clearanceBallsOverlap(double distance,double first_room,double second_room,double clearance) {
  if(!std::isfinite(distance) || !std::isfinite(first_room) || !std::isfinite(second_room) ||
     !std::isfinite(clearance) || distance<0. || clearance<0.) return false;
  const double first=std::min(2.,first_room-clearance),second=std::min(2.,second_room-clearance);
  return first>=1.e-3 && second>=1.e-3 && first+second+1.e-3>distance;
}

bool hasSupportingFloor(const Point &body,float clearance,
    const std::function<bool(const Point &)> &observed,
    const std::function<float(const Point &)> &room) {
  if(!body.allFinite() || clearance<=0.f) return false;
  // Require measured support over a footprint, with witnessed air above.
  // A tall open aisle or the edge of a thin shelf is not a floor portal.
  for(float drop=.75f*clearance;drop<=3.5f*clearance;drop+=.25f*clearance) {
    int supported=0;
    for(int x=-1;x<=1;++x) for(int y=-1;y<=1;++y) {
      const Point p=body+Point(x*clearance,y*clearance,-drop);
      const Point above=p+Point(0,0,.8f*clearance);
      if(observed(above) && room(p)<.25f*clearance && room(above)>.45f*clearance)
        ++supported;
    }
    if(supported>=6) return true;
  }
  return false;
}

bool measuredCorridorCap(const FrontierRoute &approach,const Point &direction,float clearance,
    const std::function<bool(const Point &)> &free,const std::function<float(const Point &)> &room,
    const std::function<bool(const Point &)> &surveyed) {
  if(approach.size()<2 || clearance<=0.f || direction.head<2>().norm()<.1f) return false;
  const Point side=Point(-direction.y(),direction.x(),0.f).normalized();
  float length=0.f;
  for(size_t i=1;i<approach.size();++i) length+=(approach[i]-approach[i-1]).norm();
  if(length<2.f*clearance) return false;
  // At a T junction the wall straight ahead is real. The side corridors
  // open before the wall, so examine multiple cross sections on approach.
  // An unknown lateral aperture stays available to the next incoming scan.
  for(float offset:{.75f,2.05f,3.5f,4.7f}) {
    if(offset*clearance>length+.01f) break;
    const Point center=pointAt(approach,std::max(0.f,length-offset*clearance));
    if(!free(center)) return false;
    for(float sign:{-1.f,1.f}) {
      bool accounted=false;
      float observed=0.f;
      for(float step=.35f;step<=5.3f;step+=.35f) {
        const Point p=center+(sign*step*clearance)*side;
        const float dist=room(p);
        if(dist<.44f*clearance && observed>=.35f) {accounted=true;break;}
        if(!free(p)) break;
        observed=step;
        if(step>4.8f && dist>.6f*clearance && surveyed(p)) {accounted=true;break;}
      }
      if(!accounted) return false;
    }
  }
  return true;
}

FrontierRoute observedEndTurn(const FrontierRoute &approach, const Point &heading, float radius,
    float spacing, float return_length, int side, const std::function<bool(const Point &, const Point &)> &clear,
    const Point &apex_offset) {
  if (approach.size()<2 || heading.head<2>().norm()<.5f || radius<=0.f || spacing<=0.f ||
      return_length<4.f*radius || (side!=1 && side!=-1)) return {};
  const Point h=Point(heading.x(),heading.y(),0).normalized(), lateral=side*Point(-h.y(),h.x(),0.f);
  const Point reference=approach.back(), tip=reference+apex_offset;
  float length=0.f;
  for(size_t i=1;i<approach.size();++i) length+=(approach[i]-approach[i-1]).norm();
  if(length<3.3f*radius) return {};
  const float lead=std::min(5.f*radius,length);
  FrontierRoute prefix=approach;
  auto tail=splitFrontierRoute(prefix,length-lead);
  if(tail.empty()) return {};
  for(const auto &point:tail) {
    const Point delta=point-reference;
    if(std::abs(delta.dot(lateral))>.75f*radius || std::abs(delta.z())>radius ||
        delta.dot(h)>.15f*radius) return {};
  }
  const Point cut=prefix.back(), start=tip-radius*(h+lateral), center=tip-radius*h;
  const Point exit=tip-radius*h+radius*lateral, finish=reference-return_length*h;
  auto add=[&](const Point &p) {
    if((p-prefix.back()).norm()<.005f) return true;
    if(!clear(prefix.back(),p)) return false;
    prefix.push_back(p);
    return true;
  };
  auto transition=[&](const Point &a,const Point &b) {
    const Point delta=b-a;
    const float along=delta.dot(h);
    const int n=std::max(2,int(std::ceil(delta.norm()/spacing)));
    for(int i=1;i<=n;++i) {
      const float t=float(i)/n;
      if(!add(a+t*along*h+(t*t*(3.f-2.f*t))*(delta-along*h))) return false;
    }
    return true;
  };
  if(!transition(cut,start)) return {};
  // Use an even sample count so the measured farthest point remains an
  // exact intermediate checkpoint on the published turnaround.
  const int half=std::max(3,int(std::ceil(.5f*float(M_PI)*radius/spacing)));
  for(int i=1;i<=2*half;++i) {
    const float theta=float(M_PI)*i/(2*half);
    if(!add(center+radius*(std::sin(theta)*h-std::cos(theta)*lateral))) return {};
  }
  if(!transition(exit,finish)) return {};
  return prefix;
}

FrontierRoute observedReturnLane(const Point &start,const Point &inbound,const FrontierRoute &reverse_track,
    float clearance,const std::function<bool(const Point &,const Point &)> &clear,
    const std::function<bool(const Point &)> &goal_free) {
  if(clearance<=0.f || inbound.head<2>().norm()<.5f || reverse_track.size()<2) return {};
  const Point axis=Point(inbound.x(),inbound.y(),0.f).normalized();
  FrontierRoute route{start};
  float previous=0.f;
  for(const auto &sample:reverse_track) {
    const Point delta=sample-start;
    const float along=axis.dot(delta);
    if(along<.2f*clearance && route.size()==1) continue;
    if(along<previous-.55f*clearance || std::abs(delta.z())>1.5f*clearance ||
       (delta-along*axis).head<2>().norm()>1.7f*clearance+.035f*std::max(0.f,along)) break;
    if(along<previous+.35f*clearance) continue;
    if(!clear(route.back(),sample)) break;
    route.push_back(sample); previous=along;
  }
  while(route.size()>1 && !goal_free(route.back())) route.pop_back();
  if(route.size()<2 || (route.back()-start).dot(axis)<2.5f*clearance) return {};
  // Remove tracking noise and retain bends that actually need their own
  // safe leg. Every shortcut is checked against both returns and free rays.
  FrontierRoute guide{start};
  for(size_t i=0;i+1<route.size();) {
    size_t next=i+1;
    for(size_t j=i+2;j<route.size();++j) {
      if(!clear(route[i],route[j])) break;
      next=j;
    }
    guide.push_back(route[next]);i=next;
  }
  return guide;
}

bool hasHorizontalOutlet(const Point &body, float clearance,
    const std::function<bool(const Point &)> &free, const std::function<bool(const Point &)> &unvisited) {
  if (!body.allFinite() || clearance<=0.f || !free(body)) return false;
  // Bounded flood confined to a nearly level slice. A switchback landing
  // can be open sideways but its next flight leaves this slice. Real outlets
  // can turn corners inside it. The frontier planner still makes the route.
  constexpr int radius=14, width=2*radius+1, layers=5;
  std::array<unsigned char,width*width*layers> seen{};
  struct Cell {int x,y,z;};
  std::vector<Cell> queue{{0,0,0}};
  auto key=[](Cell c) {return ((c.x+radius)*width+c.y+radius)*layers+c.z+2;};
  auto point=[&](Cell c) -> Point {return body+Point(.5f*clearance*c.x,.5f*clearance*c.y,.32f*clearance*c.z);};
  seen[key(queue.front())]=1;
  const std::array<Cell,6> steps{{{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}}};
  for(size_t i=0;i<queue.size();++i) {
    const Cell a=queue[i];
    for(const auto d:steps) {
      const Cell b{a.x+d.x,a.y+d.y,a.z+d.z};
      const int radial=b.x*b.x+b.y*b.y;
      if(std::abs(b.x)>radius || std::abs(b.y)>radius || std::abs(b.z)>2 || radial>radius*radius) continue;
      auto &tested=seen[key(b)];
      if(tested) continue;
      const Point candidate=point(b);
      if(!free(candidate)) {tested=1;continue;}
      if(!free(.5f*(point(a)+candidate))) continue;
      if(radial>=12*12 && unvisited(candidate)) return true;
      tested=1;
      queue.push_back(b);
    }
  }
  return false;
}

FrontierRoute measuredAscentTrace(const FrontierRoute &track,float clearance) {
  if(track.size()<5 || clearance<=0.f) return track;
  FrontierRoute path;
  for(const auto &point:track) {
    if(!path.empty() && (point-path.back()).norm()<.05f*clearance) continue;
    float length=0.f,low=point.z(),high=low;
    // A visit down a horizontal recess sometimes preceded a successful
    // ascent. Remove a loop only upon actually returning to the same lane
    // and height. The projected template is always recertified on reuse.
    for(size_t j=path.size();j>0;--j) {
      const auto &sample=path[j-1];
      if(j<path.size()) length+=(path[j]-sample).norm();
      low=std::min(low,sample.z());high=std::max(high,sample.z());
      if(high-low>1.15f*clearance || length>18.f*clearance) break;
      if(length>3.5f*clearance && std::abs(sample.z()-point.z())<.40f*clearance &&
         (sample-point).head<2>().norm()<.85f*clearance) {path.resize(j);break;}
    }
    if(path.empty() || (path.back()-point).norm()>.05f*clearance) path.push_back(point);
  }
  if(path.size()<4) return track;
  std::vector<float> arc{0.f};
  for(size_t i=1;i<path.size();++i) arc.push_back(arc.back()+(path[i]-path[i-1]).norm());
  size_t first=path.size(),last=0;
  for(size_t i=0;i+1<path.size();++i) {
    auto at=std::lower_bound(arc.begin()+i+1,arc.end(),arc[i]+3.f*clearance);
    if(at==arc.end()) break;
    const size_t j=at-arc.begin();
    if(path[j].z()-path[i].z()<.65f*clearance) continue;
    first=std::min(first,i);last=std::max(last,j);
  }
  if(first>=last) return path;
  // Keep measured approach and exit tangents. A long level corridor at
  // either end belongs to the floor survey and is not a stair hypothesis.
  const auto start=std::lower_bound(arc.begin(),arc.end(),arc[first]-2.2f*clearance);
  const auto end=std::upper_bound(arc.begin(),arc.end(),arc[last]+2.2f*clearance);
  if(end-start<4) return path;
  return FrontierRoute(path.begin()+(start-arc.begin()),path.begin()+(end-arc.begin()));
}

void FloorTrace::refineEntrance(const Point &position,float clearance) {
  if(trail_.size()<4 || !entrance_) return;
  FrontierRoute samples=trail_;
  if((position-samples.back()).norm()>.02f) samples.push_back(position);
  std::vector<float> arc{0.f};
  for(size_t j=1;j<samples.size();++j)
    arc.push_back(arc.back()+(samples[j]-samples[j-1]).head<2>().norm());
  if(arc.back()<6.f*clearance) return;
  // Estimate the sustained floor grade from the portion actually flown
  // away from the stairs. A low tread visible behind the landing can be
  // within one clearance radius of the hall, yet sends the return flight
  // down the wrong side of the switchback.
  std::vector<float> slopes;
  for(size_t j=0;j<samples.size();++j) {
    if(arc[j]<.5f*arc.back()) continue;
    const auto next=std::lower_bound(arc.begin()+j,arc.end(),arc[j]+2.f*clearance);
    if(next==arc.end()) break;
    const size_t k=size_t(next-arc.begin());
    slopes.push_back((samples[k].z()-samples[j].z())/(arc[k]-arc[j]));
  }
  if(slopes.empty()) return;
  std::sort(slopes.begin(),slopes.end());
  const float grade=std::clamp(slopes[slopes.size()/2],-.16f,.16f);
  std::vector<float> heights;
  for(size_t j=0;j<samples.size();++j) if(arc[j]>.5f*arc.back())
    heights.push_back(samples[j].z()-grade*arc[j]);
  std::sort(heights.begin(),heights.end());
  const float intercept=heights[heights.size()/2];
  size_t first=0;
  for(size_t j=0;j+1<samples.size() && arc[j]<.72f*arc.back();++j) {
    const auto next=std::lower_bound(arc.begin()+j,arc.end(),arc[j]+2.1f*clearance);
    if(next==arc.end()) break;
    const size_t k=size_t(next-arc.begin());
    const float slope=(samples[k].z()-samples[j].z())/(arc[k]-arc[j]);
    if(std::abs(slope-grade)>.22f ||
       std::abs(samples[j].z()-intercept-grade*arc[j])>.52f*clearance) first=j+1;
  }
  // Place the junction on the portion of this very track which genuinely
  // leaves the measured stair footprint. Never shift it to a guessed cell
  // across a wall, nor beyond the first limited horizontal departure.
  size_t portal=first;
  if(inAscentFootprint(samples[first],clearance)) {
    for(size_t j=first+1;j+1<samples.size() && arc[j]-arc[first]<6.f*clearance &&
        arc[j]<.72f*arc.back();++j) {
      if(!inAscentFootprint(samples[j],clearance) && !inAscentFootprint(samples[j+1],clearance)) {portal=j;break;}
    }
  }
  // Adjacent levels may share the same access junction. Pick it only
  // if our actual new-floor track passes that junction and is more level
  // there. Split staircases or offset floors retain their measured portal.
  if(!surveys_.empty() && !surveys_.back().departure.empty()) {
    const Point reference=surveys_.back().departure.front();
    float closest=(samples[portal]-reference).head<2>().norm();
    size_t match=portal;
    for(size_t j=first;j+1<samples.size() && arc[j]<.75f*arc.back() &&
        arc[j]-arc[first]<9.f*clearance;++j) {
      const auto next=std::lower_bound(arc.begin()+j,arc.end(),arc[j]+2.1f*clearance);
      if(next==arc.end()) break;
      const size_t k=size_t(next-arc.begin());
      if(std::abs(samples[k].z()-samples[j].z())>.23f*(arc[k]-arc[j])) continue;
      const float lateral=(samples[j]-reference).head<2>().norm();
      if(lateral<closest) {closest=lateral;match=j;}
    }
    if(closest<2.5f*clearance && (samples[portal]-reference).head<2>().norm()>closest+1.1f*clearance)
      portal=match;
  }
  portal=std::min(portal,trail_.size()-1);
  entrance_=trail_[portal];
  std::vector<float> local_heights;
  for(size_t j=portal;j<samples.size() && arc[j]-arc[portal]<1.8f*clearance;++j)
    local_heights.push_back(samples[j].z());
  std::sort(local_heights.begin(),local_heights.end());
  height_=local_heights[local_heights.size()/2];
  if(portal) trail_.erase(trail_.begin(),trail_.begin()+portal);
}

void FloorTrace::observe(const Eigen::Vector3f &p,float clearance) {
  if (!p.allFinite() || (!recent_.empty() && (p-recent_.back()).norm()<.45f*clearance)) return;
  recent_.push_back(p);
  if (recent_.size()>40) recent_.erase(recent_.begin());
  if (current_ascent_) {
    auto &trace=current_ascent_->trace;
    if (p.z()<current_ascent_->base-2.f*clearance || trace.size()>=300 ||
        (std::abs(p.z()-current_ascent_->base)<clearance && (p-trace.front()).head<2>().norm()>15.f*clearance))
      current_ascent_.reset();
    else trace.push_back(p);
  }
  if (!entrance_ || p.z()>height_+2.8f*clearance) {
    size_t begin=recent_.size()-1;
    float arc=0.f;
    while (begin>0 && std::abs(recent_[begin-1].z()-p.z())<=clearance) {
      arc+=(recent_[begin]-recent_[begin-1]).norm();
      if (arc>8.f*clearance) break;
      --begin;
    }
    if ((recent_[begin]-p).head<2>().norm()>4.5f*clearance) {
      swept_level_=false;return_completed_=false;
      access_track_.clear();access_distance_=0.f;
      entrance_=recent_[begin];
      height_=.5f*(p.z()+entrance_->z());
      if (current_ascent_ && height_>current_ascent_->base+2.5f*clearance && current_ascent_->trace.size()>5) {
        if (ascents_.size()==6) ascents_.erase(ascents_.begin());
        ascents_.push_back({current_ascent_->base,measuredAscentTrace(current_ascent_->trace,clearance)});
        current_ascent_.reset();
      }
      trail_.assign(recent_.begin()+begin,recent_.end());
      outside_=false;
    }
  }
  if (!entrance_ || std::abs(height_-p.z())>1.8f*clearance) return;
  const float far=(p-*entrance_).head<2>().norm();
  if (far<5.f*clearance) {
    // Finishing the fallback return discharges this task. Other branches
    // remain available to the exploration graph on this same level.
    if(outside_) return_completed_=true;
    outside_=false;
  }
  if (far>12.f*clearance && std::abs(height_-p.z())<1.25f*clearance) {
    outside_=true;
    if(!swept_level_) refineEntrance(p,clearance);
    if(!swept_level_ && (surveys_.empty() || height_>surveys_.back().height+2.5f*clearance)) {
      FrontierRoute departure;
      for(const auto &sample:trail_)
        if(std::abs(sample.z()-height_)<1.35f*clearance &&
           (departure.empty() || (sample-departure.back()).norm()>.7f*clearance)) departure.push_back(sample);
      departure.push_back(p);
      if(surveys_.size()>=6) surveys_.erase(surveys_.begin());
      surveys_.push_back({height_,std::move(departure)});
      access_track_=surveys_.back().departure;
      access_distance_=0.f;
      for(size_t j=1;j<access_track_.size();++j) access_distance_+=(access_track_[j]-access_track_[j-1]).norm();
    }
    swept_level_=true;
  }
  // Keep the first ordered approach long enough to include its turn into
  // the actual aisle. A stair landing can be several bends from this T.
  if(swept_level_ && !access_track_.empty() && access_distance_<28.f*clearance) {
    const float step=(p-access_track_.back()).norm();
    if(step>.45f*clearance) {access_track_.push_back(p);access_distance_+=step;}
  }
  if (trail_.empty() || (trail_.back()-p).norm()<.05f) return;
  // Erase a physically retraced loop. Separate visiting a parallel aisle
  // from meeting the old track by requiring agreement in all three axes.
  float back=0.f;
  for (size_t j=trail_.size()-1;j>0;--j) {
    back+=(trail_[j]-trail_[j-1]).norm();
    if (back>3.f*clearance && (p-trail_[j-1]).norm()<.8f*clearance) {
      trail_.resize(j);
      break;
    }
  }
  trail_.push_back(p);
}

FrontierRoute FloorTrace::outletProbes(const Point &body,float clearance) const {
  if(surveys_.empty() || clearance<=0.f) return {};
  // Use recent actually explored storeys to predict where to look. Match
  // a local adjoining shaft and tolerate differences in the measured rise.
  // The caller must see the proposed point and verify a complete connection.
  const auto &last=surveys_.back();
  if(last.departure.empty()) return {};
  std::vector<float> rises;
  for(size_t i=1;i<surveys_.size();++i) {
    const auto &a=surveys_[i-1],&b=surveys_[i];
    if(a.departure.empty() || b.departure.empty() ||
       (a.departure.front()-b.departure.front()).head<2>().norm()>10.f*clearance) continue;
    rises.push_back(b.height-a.height);
  }
  const float measured=body.z()-last.height;
  if(measured<2.5f*clearance ||
     (body-last.departure.front()).head<2>().norm()>16.f*clearance) return {};
  // For the very first ascent only the horizontal outlet on the departure
  // floor is known. Project it to the measured sensor height and require
  // this storey's own free rays and a complete connection in the caller.
  std::sort(rises.begin(),rises.end());
  const float rise=rises.empty() ? measured : rises[(rises.size()-1)/2];
  if(!rises.empty() && std::abs(measured-rise)>1.65f*clearance) return {};
  FrontierRoute proposals;
  for(auto it=last.departure.rbegin();it!=last.departure.rend();++it) {
    const Point p=*it+Point(0,0,rise);
    const float span=(p-body).head<2>().norm();
    if(span<2.6f*clearance || span>16.f*clearance ||
       std::abs(p.z()-body.z())>2.f*clearance) continue;
    if(!inAscentFootprint(p,clearance)) proposals.push_back(p);
  }
  return proposals;
}

bool FloorTrace::inAscentFootprint(const Point &p,float clearance) const {
  for(const auto &ascent:ascents_) {
    const auto &path=ascent.trace;
    for(size_t i=1;i<path.size();++i) {
      const size_t first=i>=2 ? i-2 : 0,last=std::min(i+2,path.size()-1);
      // A flat corridor at the end of a stored climb does not belong to
      // the staircase. Compare with the measured sloped portions in XY.
      if(std::abs(path[last].z()-path[first].z())<.35f*clearance) continue;
      const Eigen::Vector2f edge=(path[i]-path[i-1]).head<2>();
      const Eigen::Vector2f delta=(p-path[i-1]).head<2>();
      const float along=std::clamp(delta.dot(edge)/std::max(.0001f,edge.squaredNorm()),0.f,1.f);
      if((delta-along*edge).norm()<2.2f*clearance) return true;
    }
  }
  return false;
}

std::optional<FloorTrace::Junction> FloorTrace::lateralJunction(const Point &end,const Point &axis,float clearance) const {
  if(clearance<=0.f || !surveyedLevel(end,clearance) || access_track_.size()<3 || axis.head<2>().norm()<.5f)
    return std::nullopt;
  const Point along=Point(axis.x(),axis.y(),0.f).normalized(),side(-along.y(),along.x(),0.f);
  for(size_t j=1;j<access_track_.size();++j) {
    const Point p=access_track_[j];const float at=(p-end).dot(side);
    if(std::abs(at)>1.6f*clearance || std::abs(p.z()-end.z())>1.55f*clearance) continue;
    float arc=0.f;size_t i=j;
    while(i>0 && arc<3.6f*clearance) {arc+=(access_track_[i]-access_track_[i-1]).norm();--i;}
    const Point a=access_track_[i],edge=p-a;const float old=(a-end).dot(side),cross=edge.dot(side);
    if(std::abs(old)<2.0f*clearance || std::abs(at)>std::abs(old) || old*cross>=0.f ||
       std::abs(cross)<.54f*edge.head<2>().norm() || std::abs(cross)<.25f*clearance ||
       std::abs(a.z()-end.z())>1.65f*clearance) continue;
    const Point intersection=p-(at/cross)*edge;
    if((intersection-p).head<2>().norm()>2.5f*clearance) continue;
    const Point lane=end+(intersection-end).dot(along)*along;
    return Junction{lane,a};
  }
  return std::nullopt;
}

std::optional<FloorTrace::Junction> FloorTrace::returnJunction(const Point &end,const Point &body,
                                                                  const Point &direction,float clearance) const {
  // A short final approach skewed across the centreline gives a poor
  // bearing back to a distant junction. Fit the measured inbound straight.
  const auto track=recentReturn(body,17.f*clearance,clearance);
  if(track.size()>2) {
    Point axis=end-track.back();axis.z()=0.f;
    const float length=axis.norm();
    if(length>8.f*clearance && axis.dot(direction)>.94f*length) {
      axis/=length;
      const auto aligned=[&](const Point &p) {
        const Point d=end-p;
        return std::abs(d.z())<1.4f*clearance && (d-d.dot(axis)*axis).head<2>().norm()<1.3f*clearance;
      };
      if(std::all_of(track.begin(),track.end(),aligned))
        if(const auto junction=lateralJunction(end,axis,clearance)) return junction;
    }
  }
  return lateralJunction(end,direction,clearance);
}

bool FloorTrace::returningThroughEntrance(const FrontierRoute &remaining,float clearance) const {
  if(remaining.size()<2 || clearance<=0.f || !surveyedLevel(remaining.front(),clearance)) return false;
  if(returningThroughEntrance(remaining.front(),remaining.back(),clearance)) return true;
  if(!entrance_ || surveys_.empty() || surveys_.back().departure.size()<2) return false;
  const auto &opening=surveys_.back().departure;
  Point outward=pointAt(opening,3.8f*clearance)-opening.front();
  outward.z()=0.f;
  if(outward.norm()<1.5f*clearance) return false;
  outward.normalize();
  // A guide can turn immediately beyond the doorway. Its far endpoint
  // has a different bearing from the measured crossing itself.
  auto crosses=[&](const FrontierRoute &route) {
    float length=0.f;
    for(size_t i=1;i<route.size();++i) length+=(route[i]-route[i-1]).norm();
    for(float at=0.f;at<std::min(length,12.f*clearance)-1.6f*clearance;at+=.65f*clearance) {
      const Point a=pointAt(route,at),b=pointAt(route,std::min(length,at+3.8f*clearance));
      const float before=(a-*entrance_).dot(outward),after=(b-*entrance_).dot(outward);
      if(before<-.9f*clearance || after>1.4f*clearance || before-after<1.5f*clearance) continue;
      if(returningThroughEntrance(a,b,clearance)) return true;
    }
    return false;
  };
  if(crosses(remaining)) return true;
  const Point body=remaining.front(),ahead=pointAt(remaining,3.f*clearance);
  if((body-*entrance_).dot(outward)>1.4f*clearance || (ahead-body).dot(outward)>.8f*clearance) return false;
  // LiDAR may see an upward opening just after the vehicle has turned
  // through the doorway. Keep its recent measured crossing while it is
  // still proceeding inward. An outgoing branch is evaluated normally.
  auto recent=recentReturn(body,8.f*clearance,clearance);
  if(recent.size()<2) return false;
  std::reverse(recent.begin(),recent.end());
  auto prefix=remaining;
  splitFrontierRoute(prefix,2.5f*clearance);
  recent.insert(recent.end(),prefix.begin()+1,prefix.end());
  return crosses(recent);
}

bool FloorTrace::returningThroughEntrance(const Point &body,const Point &tip,float clearance) const {
  if(clearance<=0.f || !entrance_ || !surveyedLevel(body,clearance) || surveys_.empty()) return false;
  const auto &floor=surveys_.back();
  if(std::abs(floor.height-height_)>1.5f*clearance || floor.departure.size()<2) return false;
  Point inbound=tip-body;
  inbound.z()=0.f;
  if(inbound.norm()<.7f*clearance) return false;
  inbound.normalize();
  const Point exit=pointAt(floor.departure,3.8f*clearance)-floor.departure.front();
  if(exit.head<2>().norm()<1.5f*clearance || exit.dot(inbound)>-.72f*exit.head<2>().norm()) return false;
  const Point to=*entrance_-body;
  const float along=to.dot(inbound),dist=(tip-body).dot(inbound);
  return std::abs(to.z())<1.8f*clearance && std::abs(tip.z()-body.z())<1.5f*clearance &&
         along>=-6.f*clearance && along<=dist+2.2f*clearance &&
         (to-along*inbound).head<2>().norm()<3.5f*clearance;
}

bool FloorTrace::needsReturn(const Eigen::Vector3f &p,float clearance) const {
  return entrance_ && outside_ && !return_completed_ && std::abs(p.z()-height_)<=1.8f*clearance &&
      (p-*entrance_).head<2>().norm()>8.f*clearance;
}

std::optional<Point> FloorTrace::landingArrival(const Point &position,float clearance) const {
  if(!climbingAt(position,clearance) || recent_.size()<3) return std::nullopt;
  Point prev=position,heading=Point::Zero();
  float length=0.f;
  for(auto it=recent_.rbegin();it!=recent_.rend();++it) {
    if(it->z()>position.z()+.45f*clearance) return std::nullopt;
    length+=(*it-prev).norm();
    if(length>3.8f*clearance) return std::nullopt;
    if(length<=1.9f*clearance) heading=position-*it;
    if(position.z()-it->z()>.55f*clearance && length>1.15f*clearance) {
      heading.z()=0.f;
      if(heading.norm()<.55f*clearance) return std::nullopt;
      return heading.normalized();
    }
    prev=*it;
  }
  return std::nullopt;
}

void FloorTrace::startingAscent(const Eigen::Vector3f &p,float clearance) {
  // A stair flight can extend several metres horizontally inside its
  // shaft. That motion alone is not another completed corridor sweep.
  if (entrance_ && (p-*entrance_).head<2>().norm()<10.f*clearance) outside_=false;
  if (entrance_ && !current_ascent_ && std::abs(p.z()-height_)<2.f*clearance)
    current_ascent_=Ascent{height_,{p}};
}

std::vector<FrontierRoute> FloorTrace::ascentCandidates(const Point &p,const Point &heading,
                                                       float horizon,float clearance) const {
  // A previous storey supplies a shape hypothesis. The caller must verify
  // every part against this storey's observed volume before publishing it.
  std::vector<std::pair<float,FrontierRoute>> ranked;
  for (auto iter=ascents_.rbegin();iter!=ascents_.rend();++iter) {
    const auto &trace=iter->trace;
    const Point shift(0.f,0.f,height_-iter->base);
    if(shift.z()<2.f*clearance) continue;
    float best=std::numeric_limits<float>::infinity();
    FrontierRoute selected;
    for (size_t i=0;i+1<trace.size();++i) {
      const Point error=p-(trace[i]+shift);
      if(std::abs(error.z())>1.25f*clearance || error.head<2>().norm()>2.25f*clearance) continue;
      const float discrepancy=error.head<2>().squaredNorm()+2.f*error.z()*error.z();
      if(discrepancy>best+clearance*clearance) continue;
      FrontierRoute candidate{p};
      float length=0.f;
      for(size_t j=i+1;j<trace.size() && length<horizon;++j) {
        length+=(trace[j]-trace[j-1]).norm();
        const float blend=std::max(0.f,1.f-length/(4.f*clearance));
        const Point next=trace[j]+shift+blend*error;
        if(next.z()<p.z()-1.3f*clearance) break;
        if((next-candidate.back()).norm()>.28f*clearance) candidate.push_back(next);
      }
      if(candidate.size()<3 || candidate.back().z()<p.z()+.65f*clearance || length<2.f*clearance) continue;
      const Point first=tangent(candidate,0.f,2.f*clearance);
      const float alignment=heading.norm()>.1f ? first.dot(heading.normalized()) : 0.f;
      const float cost=discrepancy+.6f*clearance*clearance*(1.f-alignment);
      if(cost<best) {best=cost;selected=std::move(candidate);}
    }
    if(!selected.empty()) ranked.emplace_back(best,std::move(selected));
  }
  std::stable_sort(ranked.begin(),ranked.end(),[](const auto &a,const auto &b){return a.first<b.first;});
  std::vector<FrontierRoute> result;
  for(size_t i=0;i<std::min<size_t>(3,ranked.size());++i) result.push_back(std::move(ranked[i].second));
  return result;
}

FrontierRoute FloorTrace::recentReturn(const Point &body,float horizon,float clearance) const {
  if(recent_.empty() || !body.allFinite() || horizon<=0.f || clearance<=0.f ||
     (recent_.back()-body).norm()>2.f*clearance) return {};
  FrontierRoute back{body};float distance=0.f;
  for(auto it=recent_.rbegin();it!=recent_.rend();++it) {
    const float step=(*it-back.back()).norm();
    if(step<.20f*clearance) continue;
    if(distance+step>horizon || std::abs(it->z()-body.z())>2.f*clearance) break;
    back.push_back(*it);distance+=step;
  }
  return back;
}

FrontierRoute FloorTrace::returnPath(const Eigen::Vector3f &p,float horizon,float clearance) const {
  if (!needsReturn(p,clearance) || trail_.empty()) return {};
  FrontierRoute result{p};
  // Join the recent part of the actual track at the current body. A sensor
  // sample just ahead of the body must not first command a step outward.
  auto first=trail_.rbegin();
  float nearest=(*first-p).squaredNorm(), examined=0.f;
  for (auto it=trail_.rbegin()+1;it!=trail_.rend();++it) {
    examined+=(*it-*(it-1)).norm();
    if (examined>2.5f*clearance) break;
    const float d=(*it-p).squaredNorm();
    if (d<nearest+.001f) {nearest=d;first=it;}
  }
  float length=0.f;
  for (auto it=first;it!=trail_.rend();++it) {
    const float step=(*it-result.back()).norm();
    if (step>2.5f*clearance) break;
    if (step<.08f) continue;
    length+=step;
    result.push_back(*it);
    if (length>=horizon) break;
  }
  return result;
}

void CorridorPass::configure(double clearance, double confirmation, double gain_ratio) {
  if (!std::isfinite(clearance) || clearance <= 0. || !std::isfinite(confirmation) ||
      confirmation <= 0. || !std::isfinite(gain_ratio) || gain_ratio <= 1.)
    throw std::invalid_argument("Invalid EPIC corridor pass parameters");
  clearance_ = clearance;
  confirmation_ = confirmation;
  gain_ratio_ = gain_ratio;
}

void CorridorPass::observe(const Point &position) {
  if (guide_.empty() || !position.allFinite()) return;
  if ((position - trail_.back()).norm() >= .45f) {
    trail_.push_back(position);
    if (trail_.size() > 160) trail_.erase(trail_.begin());
  }
  if ((position - last_position_).squaredNorm() > 1.e-6f &&
      !remainingFrontierRoute(guide_, position, last_position_, guide_progress_).empty()) {
    const Point forward = tangent(guide_, guide_progress_, 3. * clearance_);
    if (!forward.isZero()) heading_ = forward;
  }
  last_position_ = position;
}

float CorridorPass::measuredApproach(const Point &body,const Point &direction) const {
  if(trail_.empty() || direction.head<2>().norm()<.1f) return 0.f;
  const Point axis=Point(direction.x(),direction.y(),0.f).normalized();
  float traveled=0.f;
  for(auto it=trail_.rbegin();it!=trail_.rend();++it) {
    const Point delta=body-*it;
    const float along=axis.dot(delta);
    if(along<traveled-.45f*clearance_ || std::abs(delta.z())>1.4f*clearance_ ||
       (delta-along*axis).head<2>().norm()>1.2f*clearance_+.06f*std::max(0.f,along)) break;
    traveled=std::max(traveled,along);
  }
  return traveled;
}

void CorridorPass::commit(const FrontierRoute &guide, bool reverse) {
  if (guide.size() < 2) return;
  if (guide_.empty() || reverse) {
    trail_.assign(1, guide.front());
    heading_.setZero();
    missing_since_ = -1.;
  }
  guide_ = guide;
  guide_progress_ = 0.;
  last_position_ = guide.front();
  // A tiny adjustment between two samples of one viewpoint is not a new
  // passage tangent. Keep the established direction until there is a real leg.
  const Point ahead = pointAt(guide, 3. * clearance_) - guide.front();
  if (ahead.norm() >= clearance_ || (heading_.isZero() && ahead.norm() > .25f))
    heading_ = ahead.normalized();
}

CorridorPass::Candidate CorridorPass::assessAgainst(const FrontierRoute &history,
    const FrontierRoute &path, const Point &heading, double area, const Connected &connected) const {
  Candidate result{Leg::Forward, 0., 0., area};
  if (path.size() < 2) return result;
  const double radius = 2. * clearance_, threshold = 2. * clearance_;
  const Point departure = tangent(path, 0., 3. * clearance_);
  const double alignment = departure.dot(heading);
  FrontierRoute reference = history;
  // Keep disconnected polylines separate. The odometry and the first graph
  // point can differ; a chord between the two is not a measured graph edge.
  const size_t break_at = reference.size();
  reference.push_back(path.front());
  double run = 0., longest = 0.;
  for (size_t i = 1; i < path.size(); ++i) {
    const Point edge = path[i] - path[i - 1];
    const double length = edge.norm();
    if (length < 1.e-4) continue;
    result.length += length;
    const Point direction = edge / float(length);
    const int count = std::max(1, int(std::ceil(length / (.5 * clearance_))));
    const double step = length / count;
    for (int k = 0; k < count; ++k) {
      const Point sample = path[i - 1] + direction * float((k + .5) * step);
      bool returning = false;
      for (size_t j = 1; j < reference.size() && !returning; ++j) {
        if (j == break_at) continue;
        const Point previous = reference[j] - reference[j - 1];
        const float squared = previous.squaredNorm();
        if (squared < 1.e-6f || previous.dot(direction) > -.5f * std::sqrt(squared)) continue;
        const float t = (sample - reference[j - 1]).dot(previous) / squared;
        if (t < 0.f || t > 1.f) continue;
        const Point projection = reference[j - 1] + t * previous;
        if ((projection - sample).squaredNorm() <= radius * radius)
          returning = connected(sample, projection);
      }
      run = returning ? run + step : 0.;
      longest = std::max(longest, run);
      if (returning) result.retraced += step;
    }
    reference.push_back(path[i]);
  }
  // Requiring an opposing, connected 3-D edge distinguishes a stair switchback
  // or a parallel aisle behind a wall from a return down the same passage.
  const bool leaving_backwards = alignment < -.55 &&
      (history.size() < 2 || result.retraced >= .5 * std::min(result.length, threshold));
  if (longest >= threshold || leaving_backwards) result.leg = Leg::Return;
  else if (!heading.isZero() && alignment < .5) result.leg = Leg::Branch;
  return result;
}

CorridorPass::Candidate CorridorPass::assess(const FrontierRoute &path, double area,
                                            const Connected &connected) const {
  return assessAgainst(trail_, path, heading_, area, connected);
}

bool CorridorPass::allowsPreview(const FrontierRoute &prefix, const FrontierRoute &leg,
                                bool new_pass, const Connected &connected) const {
  if (prefix.size() < 2 || leg.size() < 2) return false;
  // Check the entire preview against both its own prefix and actual travel.
  // Do not join a lagging odometry sample to the prefix through an untested chord.
  FrontierRoute backwards(prefix.rbegin(), prefix.rend());
  Point forward = -tangent(backwards, 0., 3. * clearance_);
  if (forward.isZero() && !new_pass) forward = heading_;
  if (assessAgainst(prefix, leg, forward, 0., connected).leg == Leg::Return) return false;
  FrontierRoute combined = prefix;
  combined.insert(combined.end(), leg.begin() + 1, leg.end());
  const FrontierRoute empty;
  return assessAgainst(new_pass ? empty : trail_, combined, Point::Zero(), 0., connected).leg != Leg::Return;
}

CorridorPass::Selection CorridorPass::select(const std::vector<Candidate> &candidates,
                                            double scan_stamp, bool wall) {
  Selection choice{{}, Reason::Continue};
  std::vector<size_t> branches, rear;
  for (size_t i = 0; i < candidates.size(); ++i) {
    switch (candidates[i].leg) {
      case Leg::Forward: choice.indices.push_back(i); break;
      case Leg::Branch: branches.push_back(i); break;
      case Leg::Return: rear.push_back(i); break;
    }
  }
  if (choice.indices.empty()) choice.indices.swap(branches);
  if (!choice.indices.empty()) {
    missing_since_ = -1.;
    auto rate = [&](size_t i) {
      // Visiting a rear branch also costs the distance needed to resume the pass.
      const auto &c = candidates[i];
      return c.visible_area / (std::max(2. * clearance_, c.length) + c.retraced);
    };
    auto by_rate = [&](size_t a, size_t b) { return rate(a) < rate(b); };
    if (!rear.empty()) {
      const size_t behind = *std::max_element(rear.begin(), rear.end(), by_rate);
      const size_t ahead = *std::max_element(choice.indices.begin(), choice.indices.end(), by_rate);
      if (rate(behind) > gain_ratio_ * rate(ahead)) return {{behind}, Reason::Gain};
    }
    return choice;
  }
  if (rear.empty()) return choice;
  if (wall) return {rear, Reason::Wall};
  if (missing_since_ < 0. || scan_stamp < missing_since_) missing_since_ = scan_stamp;
  if (scan_stamp - missing_since_ >= confirmation_) return {rear, Reason::NoForward};
  return {{}, Reason::AwaitScan};
}

bool CorridorPass::wallAhead(const Point &position,
                            const std::function<float(const Point &)> &distance) const {
  if (heading_.isZero()) return false;
  const Point side = heading_.unitOrthogonal();
  const Point up = heading_.cross(side);
  // Require measured returns across the passage. Missing points, a single
  // column, or a floor under a horizontal flight do not confirm a closed end.
  for (int column : {-1, 0, 1}) for (int row : {-1, 1}) {
    const Point offset = float(clearance_) * (.8f * column * side + .5f * row * up);
    bool hit = false;
    for (double s = clearance_; s <= 3. * clearance_ && !hit; s += .25 * clearance_) {
      hit = distance(position + offset + float(s) * heading_) < .35 * clearance_;
    }
    if (!hit) return false;
  }
  return true;
}
} // namespace fast_planner
