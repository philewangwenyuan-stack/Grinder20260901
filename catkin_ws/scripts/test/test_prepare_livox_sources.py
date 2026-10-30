"""Livox bootstrap: restore partial trees, preserve local files, reject bad archives."""
import hashlib
import importlib.util
import io
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[1] / 'prepare_livox_sources.py'
SPEC = importlib.util.spec_from_file_location('prepare_livox_sources', SCRIPT)
livox = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(livox)


def archive_file(path, entries):
    with tarfile.open(path, 'w:gz') as archive:
        for name, data in entries:
            entry = tarfile.TarInfo(name)
            if data is None:
                entry.type = tarfile.SYMTYPE
                entry.linkname = '/outside'
                archive.addfile(entry)
            else:
                payload = data.encode('utf-8')
                entry.size = len(payload)
                archive.addfile(entry, io.BytesIO(payload))


class PrepareLivoxSourcesTest(unittest.TestCase):
    def test_partial_tree_restored_without_overwriting_device_files(self):
        with tempfile.TemporaryDirectory(prefix='livox source ') as directory:
            root = Path(directory)
            archive = root / 'source.tar.gz'
            archive_file(archive, [('upstream/CMakeLists.txt', 'official build'),
                                   ('upstream/config/MID360s_config.json', 'upstream IP')])
            target = root / 'driver'
            (target / 'config').mkdir(parents=True)
            config = target / 'config/MID360s_config.json'
            config.write_text('device IP')
            livox.restore_missing(archive, target)
            self.assertEqual('official build', (target / 'CMakeLists.txt').read_text())
            self.assertEqual('device IP', config.read_text())
            (target / 'CMakeLists.txt').write_text('local build override')
            livox.restore_missing(archive, target)
            self.assertEqual('local build override', (target / 'CMakeLists.txt').read_text())

    def test_archive_rejected_before_any_files_written(self):
        for unsafe in ('upstream/../escape', '/absolute', 'other/root'):
            with self.subTest(member=unsafe), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                archive = root / 'bad.tar.gz'
                archive_file(archive, [('upstream/CMakeLists.txt', 'build'), (unsafe, 'bad')])
                with self.assertRaises(ValueError):
                    livox.restore_missing(archive, root / 'driver')
                self.assertFalse((root / 'driver').exists())

    def test_archive_symlink_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / 'bad.tar.gz'
            archive_file(archive, [('upstream/link', None)])
            with self.assertRaises(ValueError):
                livox.restore_missing(archive, root / 'driver')

    def test_cached_archive_checked_without_network(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'source.tar.gz'
            path.write_bytes(b'cached')
            digest = hashlib.sha256(b'cached').hexdigest()
            with patch.object(livox.urllib.request, 'urlopen', side_effect=AssertionError('network')):
                self.assertEqual(path, livox.verified_archive(path, 'https://example.invalid/', digest))
                with self.assertRaisesRegex(ValueError, 'SHA-256 mismatch'):
                    livox.verified_archive(path, 'https://example.invalid/', '0' * 64)

    def test_download_published_only_after_hash_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'source.tar.gz'
            digest = hashlib.sha256(b'official archive').hexdigest()
            with patch.object(livox.urllib.request, 'urlopen', return_value=io.BytesIO(b'wrong')):
                with self.assertRaisesRegex(ValueError, 'SHA-256 mismatch'):
                    livox.verified_archive(path, 'https://example.invalid/', digest)
            self.assertFalse(path.exists())
            with patch.object(livox.urllib.request, 'urlopen', return_value=io.BytesIO(b'official archive')):
                livox.verified_archive(path, 'https://example.invalid/', digest)
            self.assertEqual(b'official archive', path.read_bytes())

    def test_clean_restore_and_repeated_prepare_use_no_network(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            workspace = root / 'catkin_ws'
            cache = root / 'temp/grinder_livox_sources'
            cache.mkdir(parents=True)
            sources = []
            for name, revision, url, _digest, required in livox.SOURCES:
                archive = cache / (name + '-' + revision + '.tar.gz')
                entries = [('upstream/' + item, 'source') for item in required]
                entries = [(name, '<package><name>livox_ros_driver2</name>'
                            '<buildtool_depend>catkin</buildtool_depend></package>'
                            if name.endswith('package_ROS1.xml') else data) for name, data in entries]
                archive_file(archive, entries)
                sources.append((name, revision, url, hashlib.sha256(archive.read_bytes()).hexdigest(), required))
            with patch.object(livox, 'SOURCES', sources), \
                    patch.object(livox.urllib.request, 'urlopen', side_effect=AssertionError('network')):
                livox.prepare(workspace)
                livox.prepare(workspace)
            self.assertTrue((workspace / 'src/drivers/ws_livox/src/livox_ros_driver2/package.xml').is_file())


if __name__ == '__main__':
    unittest.main()
