import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import avp


class PackageTests(unittest.TestCase):
    def test_package_entry_point_delegates_to_adapter(self):
        config = Mock()
        with patch("avp.klipper.adapter.load_config") as load_adapter:
            self.assertIs(avp.load_config(config), load_adapter.return_value)
            load_adapter.assert_called_once_with(config)

    def test_package_imports_under_klipper_extras_namespace(self):
        with tempfile.TemporaryDirectory() as directory:
            extras = Path(directory) / "extras"
            extras.mkdir()
            (extras / "__init__.py").touch()
            (extras / "avp").symlink_to(
                Path(avp.__file__).resolve().parent, target_is_directory=True)
            result = subprocess.run(
                [sys.executable, "-I", "-c",
                 "import sys; sys.path.insert(0, sys.argv[1]); "
                 "import extras.avp; "
                 "from extras.avp.klipper.adapter import AVP; "
                 "from extras.avp.history.storage import History; "
                 "assert callable(extras.avp.load_config); "
                 "assert AVP.__module__ == 'extras.avp.klipper.adapter'; "
                 "assert History.__module__ == 'extras.avp.history.storage'",
                 directory],
                cwd=directory, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
