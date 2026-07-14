# Four Robot Match Supervisor

This controller runs the final assisted 2v2 physical soccer demo. It uses
Supervisor motion only for robot roots and their visible stabilizers. The
soccer ball remains a Webots physics object and is not translated, rotated,
given velocity, reset, or forced by the supervisor.

Labels are drawn with `Supervisor.setLabel`.
