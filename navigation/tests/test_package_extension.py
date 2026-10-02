"""Temporary-project packaging tests; never read or package runtime logs."""
from contextlib import redirect_stdout
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile

from navigation import check_install, package_extension


class PackageExtensionTests(unittest.TestCase):
    def test_runtime_directories_are_excluded_and_manifest_verifies_sources(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)/'source'
            extension=root/'navigation'
            sources={
                'navigation/worker.py':b'print("offline fixture")\n',
                'navigation/config/default.yaml':b'max_speed: 0.2\n',
                # Preserve the existing local.yaml packaging policy.
                'navigation/config/local.yaml':b'max_speed: 0.1\n',
                'navigation/maps/field_map.pgm':b'P5\n1 1\n255\n\r',
            }
            for name,data in sources.items():
                path=root/name
                path.parent.mkdir(parents=True,exist_ok=True)
                path.write_bytes(data)
            excluded=(
                'run/run_private/run.json',
                'run/run_private/config.yaml',
                'run/run_private/point_navigation.rviz',
                'run/run_private/node.log',
                'runs/run_private/run.json',
                'runs/run_private/events.jsonl',
                'updates/private.json',
                '__pycache__/private.json',
                '.private/session.json',
            )
            for name in excluded:
                path=extension/name
                path.parent.mkdir(parents=True,exist_ok=True)
                # CRLF in excluded content must not reach the LF check either.
                path.write_bytes(b'private runtime data\r\n')
            with patch.object(package_extension,'ROOT',root),redirect_stdout(io.StringIO()):
                package_extension.main()
            archive_path=extension/'updates/task2_point_navigation_20260926.zip'
            extracted=Path(temporary)/'installed'
            with zipfile.ZipFile(archive_path) as archive:
                self.assertIsNone(archive.testzip())
                self.assertEqual(set(archive.namelist()),set(sources)|{'navigation/package_manifest.json'})
                manifest=json.loads(archive.read('navigation/package_manifest.json'))
                self.assertEqual(manifest['files'],{
                    name:hashlib.sha256(data).hexdigest() for name,data in sources.items()
                })
                archive.extractall(extracted)
            digest_line=archive_path.with_name(archive_path.name+'.sha256').read_text(encoding='ascii')
            self.assertEqual(digest_line,hashlib.sha256(archive_path.read_bytes()).hexdigest()+'  '+archive_path.name+'\n')
            with patch.object(check_install,'ROOT',extracted),redirect_stdout(io.StringIO()):
                self.assertEqual(check_install.main(),0)
                (extracted/'navigation/worker.py').write_bytes(b'changed\n')
                self.assertEqual(check_install.main(),1)

    def test_source_text_requires_lf(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            extension=root/'navigation'
            extension.mkdir()
            (extension/'worker.py').write_bytes(b'print("CRLF")\r\n')
            with patch.object(package_extension,'ROOT',root),redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(ValueError,'Normalize this extension file to LF'):
                    package_extension.main()
            self.assertFalse((extension/'updates/task2_point_navigation_20260926.zip').exists())


if __name__=='__main__': unittest.main()
