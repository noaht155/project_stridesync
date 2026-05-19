 How to run:

  # Install dependencies
  pip install -r requirements.txt

  # Generate test data first
  cd Dashboard/sample_data
  python generate_sample.py

  # Launch dashboard
  cd ..
  python app.py
  # → open http://localhost:8050
  # → paste: sample_data/sample_run.csv into the CSV field, click Load

  What each panel does:
  - Panel 1 — 3D bilateral skeleton animation with play/pause and scrub slider (0.5× real-time)
  - Panel 2 — Knee flexion L vs R and ankle dorsiflexion L vs R over time, interactive hover
  - Panel 3 — Cadence, GCT, tibial shock, symmetry indices, fatigue comparison (first 10% vs last 10%); Garmin FIT
  comparison slot ready for when you do the outdoor validation run

  Two things to update when hardware is ready: the L_THIGH, L_SHANK, L_FOOT constants in kinematics.py (measure your own
   leg), and the FSR_THRESHOLD in metrics.py (tune to actual sensor readings).