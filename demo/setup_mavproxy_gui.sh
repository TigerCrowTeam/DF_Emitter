#!/usr/bin/env bash
set -eu

GUI_ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
GUI_VENV="$GUI_ROOT/.venv-mavproxy"

echo "Installing Ubuntu GUI support for MAVProxy (sudo password may be requested)..."
sudo apt-get install -y python3.14-venv python3-wxgtk4.0 python3-pil python3-matplotlib python3-opencv

/usr/bin/python3 -m venv --system-site-packages "$GUI_VENV"
"$GUI_VENV/bin/python" -m pip install \
    MAVProxy==1.8.74 future==1.0.0 setuptools==80.9.0

"$GUI_VENV/bin/python" - <<'PY'
import wx
import cv2
from PIL import Image
import matplotlib
from MAVProxy.modules import mavproxy_map, mavproxy_console
print("MAVProxy GUI environment READY")
print("wxPython", wx.version())
print("OpenCV", cv2.__version__)
print("Pillow", Image.__version__)
print("matplotlib", matplotlib.__version__)
PY

echo
echo "Next validation command:"
echo "  ./demo/run_mavproxy_demo.sh"
