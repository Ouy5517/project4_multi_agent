from src.soccer_framework import SoccerConfig


def test_team_one_uses_demo_display_names_without_changing_robot_topics():
    config = SoccerConfig(team_id=1)

    assert config.display_name_for_player(1) == "demo1"
    assert config.display_name_for_player(2) == "demo2"
    assert config.display_name_for_player(3) == "demo3"
    assert config.robot_names == ("robot1", "robot2", "robot3")


def test_team_two_uses_win_display_names_without_changing_robot_topics():
    config = SoccerConfig(
        team_id=2,
        robot_names=("robot4", "robot5", "robot6"),
    )

    assert config.display_name_for_player(1) == "win1"
    assert config.display_name_for_player(2) == "win2"
    assert config.display_name_for_player(3) == "win3"
    assert config.robot_names == ("robot4", "robot5", "robot6")


def test_opponent_display_names_are_from_the_other_team():
    blue_config = SoccerConfig(team_id=1)
    red_config = SoccerConfig(
        team_id=2,
        robot_names=("robot4", "robot5", "robot6"),
    )

    assert blue_config.opponent_display_name_for_player(1) == "win1"
    assert red_config.opponent_display_name_for_player(1) == "demo1"
