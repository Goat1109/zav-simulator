"""Exercise archive restoration, local-edit preservation and damaged payloads."""
import gzip
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest

source = Path(__file__).resolve().parents[1] / 'tools/super/restore_frontier_assets.py'
spec = importlib.util.spec_from_file_location('restore_frontier', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def package(root):
    directory = root / 'simulator/assets/super/frontier_package'
    directory.mkdir(parents=True)
    values = {'simulator/assets/super/frontier.pcd': b'measured point cloud',
              'simulator/assets/super/frontier_visual/textures/atlas_000.jpg': b'recorded photograph'}
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode='w') as archive:
        for name, data in values.items():
            entry = tarfile.TarInfo(name)
            entry.size = len(data)
            archive.addfile(entry, io.BytesIO(data))
    data = gzip.compress(buffer.getvalue(), mtime=0)
    chunks = []
    for index, offset in enumerate(range(0, len(data), 31)):
        part = data[offset:offset+31]
        name = f'{index:03d}.part'
        (directory / name).write_bytes(part)
        chunks.append({'name': name, 'size': len(part), 'sha256': hashlib.sha256(part).hexdigest()})
    manifest = {'archive': {'size': len(data), 'sha256': hashlib.sha256(data).hexdigest()},
                'chunks': chunks, 'files': [{'path': name, 'size': len(value),
                    'sha256': hashlib.sha256(value).hexdigest()} for name, value in values.items()]}
    (directory / 'manifest.json').write_text(json.dumps(manifest))
    return directory, manifest, values


class RestoreTest(unittest.TestCase):
    def test_restore_preserves_edits_and_force_replaces_them(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            directory, manifest, values = package(root)
            module.restore(root)
            module.restore(root, verify=True)
            name = next(iter(values))
            (root / name).write_bytes(b'local map edit')
            module.restore(root)
            self.assertEqual((root / name).read_bytes(), b'local map edit')
            with self.assertRaisesRegex(ValueError, 'verification failed'):
                module.restore(root, verify=True)
            module.restore(root, force=True)
            self.assertEqual((root / name).read_bytes(), values[name])

    def test_lfs_pointer_is_replaced_and_damaged_cache_is_rebuilt(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            directory, manifest, values = package(root)
            module.restore(root)
            name = next(iter(values))
            (root / name).write_bytes(b'version https://git-lfs.github.com/spec/v1\n')
            cache = root / '.cache/frontier-assets' / (manifest['archive']['sha256'] + '.tar.gz')
            cache.write_bytes(b'damaged cached archive')
            module.restore(root)
            module.restore(root, verify=True)

    def test_bad_chunk_fails_before_installing_files(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            directory, manifest, values = package(root)
            (directory / manifest['chunks'][0]['name']).write_bytes(b'bad download')
            with self.assertRaisesRegex(ValueError, 'checksum mismatch'):
                module.restore(root)
            self.assertTrue(all(not (root / name).exists() for name in values))

    def test_manifest_cannot_write_outside_assets(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            directory, manifest, values = package(root)
            manifest['files'][0]['path'] = '../../outside'
            (directory / 'manifest.json').write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, 'Unsafe packaged asset path'):
                module.restore(root)


if __name__ == '__main__':
    unittest.main()
