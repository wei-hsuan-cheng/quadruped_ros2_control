# Target trajectory generation (TargetManager)

The MPC target is built from the current observation and the commanded body-frame velocity coming from `/control_input`.

Given
- current base pose (world frame) \(p = [x\;\;y\;\;z]^\top\) and ZYX Euler angles \(\boldsymbol\phi = [\psi\;\;\theta\;\;\phi]^\top\) (yaw, pitch, roll),
- commanded body velocity \(v_b = [v_x\;\;v_y\;\;v_z]^\top\) and yaw rate \(\omega_z\),
- time-to-target \(T = \texttt{mpc.timeHorizon}\),
- commanded stand height \(h = \texttt{comHeight}\),

TargetManager computes
\[
R = R_{zyx}(\boldsymbol\phi), \qquad v_w = R\,v_b
\]
\[
p^\ast = p + v_w\,T, \qquad
z^\ast = h, \qquad
\psi^\ast = \psi + \omega_z\,T, \qquad
\theta^\ast = 0, \quad \phi^\ast = 0
\]

The target pose stacked as \(\boldsymbol{q}^\ast = [p_x^\ast\;p_y^\ast\;z^\ast\;\psi^\ast\;0\;0]^\top\) is paired with the reach time
\[
t^\ast = t_{\text{now}} + T
\]

Two knots are sent to the reference manager:
- positions: both knots use \(\boldsymbol{q}^\ast\),
- velocities: the first two state waypoints have their linear velocity set to \(v_w\) (the rotated command).

Code reference: `controllers/ocs2_quadruped_controller/src/control/TargetManager.cpp`.
