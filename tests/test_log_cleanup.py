"""Exercise the installer-generated cleanup script against temporary logs."""

import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import time
import unittest


class LogCleanupTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="gps cleanup ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.home = self.root / "service home"
        self.logs = self.home / ".ros/log"
        self.logs.mkdir(parents=True)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        # Isolate PATH so missing fuser can be tested on any host.
        for command in ("bash", "cut", "find", "rm"):
            (self.bin / command).symlink_to(shutil.which(command))
        self.stub("getent", 'printf "gpstrust:x:1000:1000::%s:/bin/bash\\n" "$TEST_HOME"\n')
        if shutil.which("fuser"):
            (self.bin / "fuser").symlink_to(shutil.which("fuser"))
        installer = (Path(__file__).resolve().parents[1] / "setup/install_gpstrust_service.sh").read_text()
        body = installer.split('cat > "$CLEANUP_SCRIPT" <<\'EOF\'\n', 1)[1].split("\nEOF", 1)[0]
        # Never source the host configuration or touch its logs.
        body = body.replace("/etc/gpstrust.env", shlex.quote(str(self.root / "unused.env")))
        self.script = self.root / "cleanup.sh"
        self.script.write_text(body)
        self.env = dict(os.environ, PATH=str(self.bin), TEST_HOME=str(self.home),
                        LOG_DIR=str(self.root / "wrapper logs"),
                        LOG_RETENTION_DAYS="7", SERVICE_USER="gpstrust")

    def stub(self, command, body):
        path = self.bin / command
        path.unlink(missing_ok=True)
        path.write_text("#!/bin/bash\n" + body)
        path.chmod(0o755)

    def age(self, path):
        old = time.time() - 10 * 86400
        os.utime(path, (old, old))

    def log(self, name, old=True):
        path = self.logs / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("log\n")
        if old:
            self.age(path)
        self.age(path.parent)
        return path

    def run_cleanup(self):
        return subprocess.run(["/bin/bash", str(self.script)], env=self.env,
                              capture_output=True, text=True, check=True)

    def require_fuser(self):
        if not shutil.which("fuser"):
            self.skipTest("psmisc/fuser required for live-file tests")

    def test_multiple_old_active_runs_and_lazy_log_directory_survive(self):
        self.require_fuser()
        first = self.log("first run/launch.log")
        second = self.log("second run/nested/launch.log")
        lazy = self.logs / "empty active run"
        lazy.mkdir()
        self.age(lazy)
        with first.open("a") as one, second.open("a") as two:
            self.run_cleanup()
            self.assertTrue(first.exists())
            self.assertTrue(second.exists())
            one.write("still running\n")
            two.write("still running\n")
            # A delayed logger must still be able to open its log file.
            (lazy / "launch.log").write_text("first message\n")

    def test_expired_closed_files_removed_but_directories_and_recent_logs_kept(self):
        self.require_fuser()
        expired = self.log("old run/nested/expired.log")
        recent = self.log("old run/recent.log", old=False)
        self.run_cleanup()
        self.assertFalse(expired.exists())
        self.assertTrue(expired.parent.is_dir())
        self.assertTrue(recent.exists())

    def test_symlink_target_outside_ros_logs_is_untouched(self):
        self.require_fuser()
        outside = self.root / "outside.log"
        outside.write_text("keep\n")
        self.age(outside)
        (self.logs / "linked.log").symlink_to(outside)
        self.run_cleanup()
        self.assertTrue(outside.exists())

    def test_missing_fuser_preserves_expired_logs(self):
        (self.bin / "fuser").unlink(missing_ok=True)
        old = self.log("old run/launch.log")
        result = self.run_cleanup()
        self.assertTrue(old.exists())
        self.assertIn("Skipping ROS log cleanup", result.stderr)

    def test_fuser_error_preserves_expired_logs(self):
        self.stub("fuser", "exit 2\n")
        old = self.log("old run/launch.log")
        result = self.run_cleanup()
        self.assertTrue(old.exists())
        self.assertIn("fuser failed", result.stderr)

    def test_empty_and_missing_ros_log_directory(self):
        self.run_cleanup()
        self.logs.rmdir()
        self.run_cleanup()


if __name__ == "__main__":
    unittest.main()
