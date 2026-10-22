"""Real production host lifecycle, with a test-only USB backend and no hardware."""
import os
import subprocess
import sys

binary, backend = sys.argv[1:]
for mode in (None, "WBR_MOCK_BAD_CAPS", "WBR_MOCK_OLD_FIRMWARE", "WBR_MOCK_DISCONNECT", "WBR_MOCK_SAFETY_REJECT", "WBR_MOCK_ARM_REJECT", "WBR_MOCK_V1_FIRMWARE", "WBR_MOCK_DATA_REJECT"):
    environment = dict(os.environ, LD_PRELOAD=backend, WBR_MOCK_REQUIRE_PREPOST="1")
    if mode:
        environment[mode] = "1"
    result = subprocess.run([binary, "--no-rt", "--seconds", "0.4"], env=environment,
                            capture_output=True, text=True, timeout=8)
    expected = 0 if mode is None else 1
    assert result.returncode == expected, (mode, result.stdout, result.stderr, result.returncode)
    if mode is None:
        assert "output=disabled" in result.stdout and "Stopped;" in result.stdout
    elif mode == "WBR_MOCK_OLD_FIRMWARE":
        assert "handshake timed out" in result.stderr
    elif mode == "WBR_MOCK_BAD_CAPS":
        assert "capability missing" in result.stderr
    elif mode in ("WBR_MOCK_SAFETY_REJECT", "WBR_MOCK_ARM_REJECT"):
        assert "rejected safety" in result.stderr
    elif mode == "WBR_MOCK_V1_FIRMWARE":
        assert "version mismatch" in result.stderr
    elif mode == "WBR_MOCK_DATA_REJECT":
        assert "rejected a data request" in result.stderr
    else:
        assert "USB IN status=5 actual_length=0 expected_length=64" in result.stderr
print("Production handshake, merged commands, sensor reception, stop, old firmware rejection and disconnect passed")
