"""
StrideSync post-run gait analysis dashboard.
Run:  python app.py
Open: http://localhost:8050
"""
from __future__ import annotations

import io
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import plotly.subplots as sp
import dash
from dash import dcc, html, Input, Output, State

import garmin_compare
from data_loader import load_csv, pivot_wide
from kinematics import compute_joint_angles, get_skeleton_frame
from metrics import summary_metrics

# ── App init ──────────────────────────────────────────────────────────────────

app = dash.Dash(__name__, title='StrideSync Gait Analysis')
app.config.suppress_callback_exceptions = True

_FONT_MONO   = {'fontFamily': 'monospace'}
_PANEL_STYLE = {'padding': '20px 24px', 'borderBottom': '1px solid #ddd'}

# ── Layout ────────────────────────────────────────────────────────────────────

app.layout = html.Div([

    html.H1('StrideSync Gait Analysis',
            style={**_FONT_MONO, 'textAlign': 'center', 'padding': '20px 0 4px'}),
    html.P('Bilateral running gait — IMU + FSR post-run analysis',
           style={'textAlign': 'center', 'color': '#666', 'marginBottom': '20px'}),

    # ── File loader ───────────────────────────────────────────────────────────
    html.Div([
        html.Div([
            html.Label('Run CSV', style=_FONT_MONO),
            dcc.Input(id='csv-path', type='text',
                      placeholder='Path to run CSV (or sample_data/sample_run.csv)…',
                      style={'width': '100%', 'padding': '6px', 'boxSizing': 'border-box'}),
        ], style={'flex': '3', 'marginRight': '12px'}),
        html.Div([
            html.Label('Garmin .fit (optional)', style=_FONT_MONO),
            dcc.Input(id='fit-path', type='text',
                      placeholder='Path to .fit file…',
                      style={'width': '100%', 'padding': '6px', 'boxSizing': 'border-box'}),
        ], style={'flex': '2', 'marginRight': '12px'}),
        html.Div([
            html.Label(' ', style=_FONT_MONO),
            html.Button('Load Run', id='load-btn', n_clicks=0,
                        style={'width': '100%', 'padding': '7px', 'cursor': 'pointer'}),
        ], style={'flex': '0 0 100px'}),
    ], style={'display': 'flex', 'alignItems': 'flex-end', 'padding': '0 24px 8px'}),

    html.Div(id='load-status',
             style={'color': '#c00', 'paddingLeft': '24px', 'minHeight': '20px',
                    'fontFamily': 'monospace', 'fontSize': '13px'}),

    # Hidden data store
    dcc.Store(id='run-data'),

    # ── Panel 1: 3D skeleton animation ───────────────────────────────────────
    # Play / Pause / scrub slider are embedded in the Plotly figure itself —
    # all animation runs client-side after the initial load.
    html.Div([
        html.H3('Panel 1 — 3D Skeleton Animation', style=_FONT_MONO),
        html.P('Use the Play button and scrub bar inside the chart to control playback.',
               style={'color': '#888', 'fontSize': '12px', 'margin': '0 0 8px'}),
        dcc.Loading(
            dcc.Graph(id='skeleton-graph', style={'height': '560px'},
                      config={'displayModeBar': False}),
            type='circle',
        ),
    ], style=_PANEL_STYLE),

    # ── Panel 2: Joint angle time series ─────────────────────────────────────
    html.Div([
        html.H3('Panel 2 — Joint Angles', style=_FONT_MONO),
        dcc.Loading(
            dcc.Graph(id='joint-angle-graph', style={'height': '420px'},
                      config={'displayModeBar': True}),
            type='circle',
        ),
    ], style=_PANEL_STYLE),

    # ── Panel 3: Metrics + Garmin comparison ─────────────────────────────────
    html.Div([
        html.H3('Panel 3 — Gait Metrics', style=_FONT_MONO),
        html.Div(id='metrics-section'),
        html.H4('Garmin Comparison', style={**_FONT_MONO, 'marginTop': '20px'}),
        html.Div(id='garmin-section'),
    ], style={**_PANEL_STYLE, 'borderBottom': 'none'}),

], style={'maxWidth': '1100px', 'margin': '0 auto', 'fontFamily': 'sans-serif'})


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_skeleton_traces(pts_left, pts_right):
    """Return two Scatter3d traces for one frame."""
    traces = []
    for pts, name, color in [(pts_left, 'Left', '#1f77b4'), (pts_right, 'Right', '#e07020')]:
        if pts is None:
            traces.append(go.Scatter3d(x=[], y=[], z=[], mode='lines+markers',
                                       name=name, showlegend=True,
                                       line=dict(color=color, width=7),
                                       marker=dict(size=6, color=color)))
        else:
            hip, knee, ankle, toe = pts
            xs = [p[0] for p in (hip, knee, ankle, toe)]
            ys = [p[1] for p in (hip, knee, ankle, toe)]
            zs = [p[2] for p in (hip, knee, ankle, toe)]
            traces.append(go.Scatter3d(x=xs, y=ys, z=zs, mode='lines+markers',
                                       name=name, showlegend=True,
                                       line=dict(color=color, width=7),
                                       marker=dict(size=6, color=color)))
    return traces


def build_skeleton_animation(wide_df: pd.DataFrame, stride: int = 8) -> go.Figure:
    """
    Build a Plotly figure with embedded animation frames.
    All playback is client-side — no server round-trips per frame.

    stride=8 at 200 Hz source → 25 fps display animation.
    """
    indices = list(range(0, len(wide_df), stride))
    time_s  = wide_df['time_s'].values if 'time_s' in wide_df.columns \
              else np.arange(len(wide_df)) / 200.0

    # Build Plotly frames
    plotly_frames = []
    for i in indices:
        left, right = get_skeleton_frame(wide_df, i)
        t = float(time_s[i]) if i < len(time_s) else i / 200.0
        plotly_frames.append(go.Frame(
            data=_make_skeleton_traces(left, right),
            name=f'{t:.3f}',
        ))

    # Slider steps (one per frame)
    slider_steps = []
    for idx, i in enumerate(indices):
        t = float(time_s[i]) if i < len(time_s) else i / 200.0
        slider_steps.append(dict(
            method='animate',
            args=[[f'{t:.3f}'], {
                'mode': 'immediate',
                'frame': {'duration': 0, 'redraw': True},
                'transition': {'duration': 0},
            }],
            label=f'{t:.1f}s',
        ))

    frame_duration_ms = int(1000 / 25)  # 25 fps

    initial_left, initial_right = get_skeleton_frame(wide_df, 0) \
        if len(wide_df) > 0 else (None, None)

    fig = go.Figure(
        data=_make_skeleton_traces(initial_left, initial_right),
        frames=plotly_frames,
        layout=go.Layout(
            # Play / Pause buttons embedded in figure
            updatemenus=[dict(
                type='buttons',
                showactive=False,
                y=1.12,
                x=0.0,
                xanchor='left',
                yanchor='top',
                pad=dict(t=0, r=10),
                buttons=[
                    dict(
                        label='▶  Play',
                        method='animate',
                        args=[None, {
                            'frame': {'duration': frame_duration_ms, 'redraw': True},
                            'fromcurrent': True,
                            'transition': {'duration': 0},
                            'mode': 'immediate',
                        }],
                    ),
                    dict(
                        label='⏸  Pause',
                        method='animate',
                        args=[[None], {
                            'mode': 'immediate',
                            'frame': {'duration': 0, 'redraw': False},
                            'transition': {'duration': 0},
                        }],
                    ),
                ],
            )],
            # Scrub slider embedded in figure
            sliders=[dict(
                active=0,
                steps=slider_steps,
                x=0.0,
                y=0.0,
                len=1.0,
                xanchor='left',
                yanchor='top',
                pad=dict(b=10, t=50),
                currentvalue=dict(
                    prefix='Time: ',
                    visible=True,
                    xanchor='right',
                    font=dict(size=12, color='#555'),
                ),
                transition=dict(duration=0),
            )],
            scene=dict(
                # Z is forward/back (sagittal swing). Toe can reach Z≈-0.85m
                # at peak plantarflexion, so use ±1.1m to never clip.
                xaxis=dict(range=[-0.6,  0.6],  title='X (m)'),
                yaxis=dict(range=[-1.5,  0.3],  title='Y (m)'),
                zaxis=dict(range=[-1.1,  1.1],  title='Z (m, fwd)'),
                aspectmode='manual',
                aspectratio=dict(x=1.0, y=1.5, z=1.8),
                camera=dict(
                    eye=dict(x=0.3, y=0.25, z=2.5),
                    up=dict(x=0, y=1, z=0),
                ),
            ),
            margin=dict(l=0, r=0, t=60, b=0),
            legend=dict(x=0.85, y=0.98),
        ),
    )
    return fig


def _metrics_table(m: dict) -> html.Table:
    label_map = [
        ('cadence_L',            'Cadence — Left (spm)'),
        ('cadence_R',            'Cadence — Right (spm)'),
        ('gct_ms_L',             'Ground Contact Time — Left (ms)'),
        ('gct_ms_R',             'Ground Contact Time — Right (ms)'),
        ('tibial_shock_L_g',     'Tibial Shock — Left (g)'),
        ('tibial_shock_R_g',     'Tibial Shock — Right (g)'),
        ('knee_symmetry_pct',    'Knee Symmetry Index (%)'),
        ('ankle_symmetry_pct',   'Ankle Symmetry Index (%)'),
        ('cadence_symmetry_pct', 'Cadence Symmetry Index (%)'),
    ]
    fatigue = m.get('fatigue', {})
    rows = []
    for key, label in label_map:
        val = m.get(key)
        rows.append(html.Tr([
            html.Td(label, style={'paddingRight': '32px', 'paddingBottom': '4px', 'color': '#444'}),
            html.Td(str(val) if val is not None else '—',
                    style={'fontWeight': 'bold', 'paddingBottom': '4px'}),
        ]))
    if fatigue:
        rows.append(html.Tr([html.Td(html.Hr(), colSpan=2)]))
        for label, key in [
            ('Fatigue: mean knee angle — first 10%', 'first_deg'),
            ('Fatigue: mean knee angle — last 10%',  'last_deg'),
        ]:
            v = fatigue.get(key)
            rows.append(html.Tr([
                html.Td(label, style={'paddingRight': '32px', 'color': '#444'}),
                html.Td(f'{v} °' if v is not None else '—', style={'fontWeight': 'bold'}),
            ]))
        delta = fatigue.get('delta_deg')
        rows.append(html.Tr([
            html.Td('Fatigue delta (last − first)', style={'paddingRight': '32px', 'color': '#444'}),
            html.Td(f'{delta} °' if delta is not None else '—',
                    style={'fontWeight': 'bold',
                           'color': '#c00' if (delta or 0) > 2 else '#080'}),
        ]))
    return html.Table([html.Tbody(rows)],
                      style={**_FONT_MONO, 'fontSize': '13px', 'borderCollapse': 'collapse'})


# ── Callbacks ─────────────────────────────────────────────────────────────────

@app.callback(
    Output('run-data',    'data'),
    Output('load-status', 'children'),
    Input('load-btn',     'n_clicks'),
    State('csv-path',     'value'),
    prevent_initial_call=True,
)
def load_run(_, csv_path):
    if not csv_path:
        return None, 'Enter a CSV path.'
    path = Path(csv_path)
    if not path.exists():
        return None, f'File not found: {csv_path}'
    try:
        df_raw = load_csv(path)
        wide   = pivot_wide(df_raw)
        angles = compute_joint_angles(wide)
        wide   = pd.concat([wide, angles], axis=1)
        return wide.to_json(date_format='iso', orient='split'), ''
    except Exception as exc:
        return None, f'Load error: {exc}'


@app.callback(
    Output('skeleton-graph', 'figure'),
    Input('run-data',        'data'),
    prevent_initial_call=True,
)
def build_skeleton(data_json):
    if not data_json:
        return go.Figure()
    wide = pd.read_json(io.StringIO(data_json), orient='split')
    # stride=8 at 200 Hz → 25 fps smooth client-side animation
    return build_skeleton_animation(wide, stride=8)


@app.callback(
    Output('joint-angle-graph', 'figure'),
    Input('run-data',           'data'),
    prevent_initial_call=True,
)
def update_joint_angles(data_json):
    if not data_json:
        return go.Figure()
    wide   = pd.read_json(io.StringIO(data_json), orient='split')
    time_s = wide['time_s'] if 'time_s' in wide.columns else wide.index / 200.0

    fig = sp.make_subplots(
        rows=2, cols=1,
        shared_xaxes=True,
        subplot_titles=['Knee Flexion (°)', 'Ankle Dorsiflexion (°)'],
        vertical_spacing=0.12,
    )
    for col, name, color, row in [
        ('knee_L',  'Knee Left',   '#1f77b4', 1),
        ('knee_R',  'Knee Right',  '#e07020', 1),
        ('ankle_L', 'Ankle Left',  '#2ca02c', 2),
        ('ankle_R', 'Ankle Right', '#d62728', 2),
    ]:
        if col in wide.columns:
            fig.add_trace(
                go.Scatter(x=time_s, y=wide[col], name=name,
                           line=dict(color=color, width=1.5), opacity=0.85),
                row=row, col=1,
            )
    fig.update_xaxes(title_text='Time (s)', row=2, col=1)
    fig.update_yaxes(title_text='Degrees', row=1, col=1)
    fig.update_yaxes(title_text='Degrees', row=2, col=1)
    fig.update_layout(margin=dict(t=40, b=10), hovermode='x unified',
                      legend=dict(x=1.01, y=1))
    return fig


@app.callback(
    Output('metrics-section', 'children'),
    Output('garmin-section',  'children'),
    Input('run-data',         'data'),
    State('fit-path',         'value'),
    prevent_initial_call=True,
)
def update_metrics(data_json, fit_path):
    if not data_json:
        return 'No data loaded.', ''
    wide  = pd.read_json(io.StringIO(data_json), orient='split')
    m     = summary_metrics(wide)
    table = _metrics_table(m)

    garmin_out = html.P('No Garmin .fit file provided.', style={'color': '#888'})
    if fit_path:
        fp = Path(fit_path)
        if not fp.exists():
            garmin_out = html.P(f'File not found: {fit_path}', style={'color': '#c00'})
        else:
            try:
                gdf   = garmin_compare.load_fit(fp)
                gm    = garmin_compare.extract_garmin_metrics(gdf)
                c_cmp = garmin_compare.compare_cadence(gm, m)
                g_cmp = garmin_compare.compare_gct(gm, m)
                g_rows = []
                for label, cmp in [('Cadence', c_cmp), ('Ground Contact Time', g_cmp)]:
                    if cmp is None:
                        continue
                    keys = list(cmp.keys())
                    g_rows.append(html.Tr([
                        html.Td(label,                           style={'paddingRight': '24px', 'color': '#444'}),
                        html.Td(f'Garmin: {cmp[keys[0]]}',      style={'paddingRight': '16px'}),
                        html.Td(f'StrideSync: {cmp[keys[1]]}',  style={'paddingRight': '16px'}),
                        html.Td(f'Agreement: {cmp["agreement_pct"]} %',
                                style={'fontWeight': 'bold',
                                       'color': '#080' if cmp['agreement_pct'] >= 95 else '#c60'}),
                    ]))
                garmin_out = html.Table([html.Tbody(g_rows)],
                                        style={**_FONT_MONO, 'fontSize': '13px'}) \
                    if g_rows else html.P('No matching Garmin metrics found.',
                                         style={'color': '#888'})
            except Exception as exc:
                garmin_out = html.P(f'Garmin error: {exc}', style={'color': '#c00'})

    return table, garmin_out


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    app.run(debug=True, port=8050)
