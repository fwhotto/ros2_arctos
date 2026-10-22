#!/usr/bin/env python3
"""Offline acquisition checks; no CAD kernel or network required."""

import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile

import inspect_voron_cad as inspection


class AcquisitionTests(unittest.TestCase):
    def test_checksum_rejects_changed_file_without_overwriting(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "asset"
            path.write_bytes(b"original")
            digest = hashlib.sha256(b"original").hexdigest()
            self.assertEqual(inspection.verified_bytes(path, digest), b"original")
            path.write_bytes(b"user changes")
            with self.assertRaises(ValueError):
                inspection.verified_bytes(path, digest)
            self.assertEqual(path.read_bytes(), b"user changes")

    def test_existing_archive_extracts_only_approved_member(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            archive_path = root / "fixture.zip"
            content = b"fixture STEP"
            with zipfile.ZipFile(archive_path, "w") as archive:
                archive.writestr(inspection.STEP_NAME, content)
                archive.writestr("unexpected", b"not approved")
            assets = (("unused", archive_path.name,
                       hashlib.sha256(archive_path.read_bytes()).hexdigest()),)
            with patch.object(inspection, "ASSETS", assets), \
                    patch.object(inspection, "STEP_SHA256", hashlib.sha256(content).hexdigest()), \
                    patch.object(inspection, "urlopen", side_effect=AssertionError("Network used")):
                step = inspection.fetch_sources(root)
                self.assertEqual(step.read_bytes(), content)
                self.assertFalse((root / "unexpected").exists())
                self.assertEqual(inspection.fetch_sources(root), step)
                step.write_bytes(b"local changes")
                with self.assertRaises(ValueError):
                    inspection.fetch_sources(root)
                self.assertEqual(step.read_bytes(), b"local changes")


if __name__ == "__main__":
    unittest.main()
