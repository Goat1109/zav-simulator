#!/usr/bin/env python3
"""Restore the checked-in Frontier package without Git LFS or third-party modules."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import tarfile


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()


def destination(root, name):
    relative = PurePosixPath(name)
    if relative.is_absolute() or '..' in relative.parts or relative.parts[:3] != ('simulator', 'assets', 'super'):
        raise ValueError(f'Unsafe packaged asset path: {name}')
    path = root.joinpath(*relative.parts)
    if not path.parent.resolve().is_relative_to(root.resolve()):
        raise ValueError(f'Packaged asset escapes workspace: {name}')
    return path


def available(path):
    if not path.is_file():
        return False
    with path.open('rb') as stream:
        return not stream.read(100).startswith(b'version https://git-lfs.github.com/spec/v1\n')


def archive_path(root, package, manifest):
    archive = manifest['archive']
    cache = root / '.cache/frontier-assets'
    cache.mkdir(parents=True, exist_ok=True)
    path = cache / (archive['sha256'] + '.tar.gz')
    if path.is_file() and path.stat().st_size == archive['size'] and digest(path) == archive['sha256']:
        return path
    temporary = path.with_suffix('.tmp')
    try:
        with temporary.open('wb') as output:
            for chunk in manifest['chunks']:
                if Path(chunk['name']).name != chunk['name']:
                    raise ValueError('Unsafe asset chunk filename')
                source = package / chunk['name']
                if source.stat().st_size != chunk['size'] or digest(source) != chunk['sha256']:
                    raise ValueError(f'Frontier package checksum mismatch: {source.name}')
                with source.open('rb') as stream:
                    shutil.copyfileobj(stream, output, 8 * 1024 * 1024)
        if temporary.stat().st_size != archive['size'] or digest(temporary) != archive['sha256']:
            raise ValueError('Frontier archive checksum mismatch')
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
    return path


def restore(root, force=False, verify=False):
    root = Path(root).resolve()
    package = root / 'simulator/assets/super/frontier_package'
    manifest = json.loads((package / 'manifest.json').read_text())
    files = {entry['path']: entry for entry in manifest['files']}
    if len(files) != len(manifest['files']):
        raise ValueError('Duplicate asset manifest entries')
    paths = {name: destination(root, name) for name in files}
    if verify:
        failed = [name for name, path in paths.items()
                  if not available(path) or path.stat().st_size != files[name]['size'] or digest(path) != files[name]['sha256']]
        if failed:
            raise ValueError('Frontier asset verification failed: ' + ', '.join(failed))
        print(f'[Frontier] Verified {len(files)} packaged assets')
        return
    pending = {name for name, path in paths.items() if force or not available(path)}
    if not pending:
        return  # Preserve existing local maps and mission edits.
    print(f'[Frontier] Restoring {len(pending)} assets from the bundled map...', flush=True)
    archive = archive_path(root, package, manifest)
    seen = set()
    with tarfile.open(archive, 'r:gz') as source:
        for member in source:
            if not member.isfile() or member.name not in files or member.name in seen:
                raise ValueError(f'Unexpected archive entry: {member.name}')
            seen.add(member.name)
            if member.name not in pending:
                continue
            entry, target = files[member.name], paths[member.name]
            if member.size != entry['size']:
                raise ValueError(f'Archive entry size mismatch: {member.name}')
            target.parent.mkdir(parents=True, exist_ok=True)
            temporary = target.with_name(target.name + '.restore.tmp')
            try:
                with source.extractfile(member) as stream, temporary.open('wb') as output:
                    shutil.copyfileobj(stream, output, 8 * 1024 * 1024)
                if digest(temporary) != entry['sha256']:
                    raise ValueError(f'Asset checksum mismatch: {member.name}')
                temporary.chmod(entry.get('mode', 0o644))
                temporary.replace(target)
            finally:
                temporary.unlink(missing_ok=True)
    if seen != set(files):
        raise ValueError('Frontier archive is missing manifest entries')
    print('[Frontier] Runtime assets restored', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify', action='store_true', help='Check every installed asset against the packaged version')
    parser.add_argument('--force', action='store_true', help='Replace existing assets and mission files with the packaged version')
    args = parser.parse_args()
    restore(Path(__file__).resolve().parents[3], args.force, args.verify)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, tarfile.TarError) as error:
        raise SystemExit(f'[Frontier] {error}')
