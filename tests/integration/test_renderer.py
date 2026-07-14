import os

from visualization.field_renderer import render_png_from_trajectory


def test_headless_renderer_writes_nonblank_png(tmp_path):
    os.environ["MPLBACKEND"] = "Agg"
    trajectory = tmp_path / "trajectory.csv"
    trajectory.write_text(
        "\n".join(
            [
                "tick,timestamp,entity,robot_id,x,y,vx,vy,state,role",
                "1,0.0,robot,0,-1.0,0.0,,,CHASE,ball_carrier",
                "1,0.0,robot,1,-2.0,1.0,,,RECEIVE,supporter",
                "1,0.0,robot,10,1.0,0.0,,,IDLE,",
                "1,0.0,ball,,0.0,0.0,0.0,0.0,,",
            ]
        ),
        encoding="utf-8",
    )
    output = tmp_path / "frame.png"

    render_png_from_trajectory(trajectory, output)

    assert output.is_file()
    assert output.stat().st_size > 10_000
