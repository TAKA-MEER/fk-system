"""ホストの pytest から fk_auto を import できるようにする（fk_ws/src/fk_auto を sys.path に足す）。"""
import os
import sys

_PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)
