# Changelog

## 2026-06-17
- Dashboard: created a `venv` and installed `requirements.txt` (numpy, pandas, matplotlib, plotly, dash, requests, fitparse) to fix `ModuleNotFoundError: No module named 'numpy'` when running `app.py` on a system with an externally-managed Python.
- Verified no other missing dependencies in Dashboard or ESP32 (ESP32's `lib_deps` in `platformio.ini` already cover everything `main.cpp` uses).
- Added `venv/` to `.gitignore`.
- Removed stray tracked `Dashboard/errors.txt` (large runtime debug log, same class of issue as the earlier `debug.log` cleanup) and added `errors.txt` to `.gitignore`.
