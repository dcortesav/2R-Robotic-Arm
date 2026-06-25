import numpy as np
import matplotlib.pyplot as plt

def solve_spline(x0, y0, x1, y1, x2, y2):
    """Resuelve la matriz de coeficientes para la spline cúbica natural C^2."""
    A = np.array([
        [x0**3, x0**2, x0, 1, 0, 0, 0, 0],
        [x1**3, x1**2, x1, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, x1**3, x1**2, x1, 1],
        [0, 0, 0, 0, x2**3, x2**2, x2, 1],
        [3*x1**2, 2*x1, 1, 0, -3*x1**2, -2*x1, -1, 0],
        [6*x1, 2, 0, 0, -6*x1, -2, 0, 0],
        [6*x0, 2, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 6*x2, 2, 0, 0]
    ])
    b = np.array([y0, y1, y1, y2, 0, 0, 0, 0])
    coef = np.linalg.solve(A, b)
    return coef

# ========================================================
# 1) PARÁMETROS CONFIGURABLES
# ========================================================
lado_base = 20e-2
factor_escalamiento = 1.2
lado = lado_base*factor_escalamiento          # [m] Tamaño del lado objetivo
phi_deg = 45.0        # [grados] Rotación del trébol

N_clover = 300         # Número de puntos discretos del trébol
N_approach = 50       # Número de puntos discretos de aproximación

# Fijos
x_start = 5e-2
y_start = 0
epsilon = 0.2
n_curvas = 4
v_cart = 10e-2

phi = np.radians(phi_deg)

L1 = 0.24
L2 = 0.19

# ========================================================
# 2) GENERACIÓN GEOMÉTRICA
# ========================================================
N_cont = 1000
theta_cont = np.linspace(0, 2*np.pi, N_cont)
abs_suave_cont = np.sqrt(np.cos((n_curvas/2) * theta_cont)**2 + epsilon**2)
x_raw_cont = (0.99 + abs_suave_cont) * np.cos(theta_cont)
y_raw_cont = (0.99 + abs_suave_cont) * np.sin(theta_cont)

theta_disc = np.linspace(0, 2*np.pi, N_clover)
abs_suave_disc = np.sqrt(np.cos((n_curvas/2) * theta_disc)**2 + epsilon**2)
x_raw_disc = (0.99 + abs_suave_disc) * np.cos(theta_disc)
y_raw_disc = (0.99 + abs_suave_disc) * np.sin(theta_disc)

# A) Centrar la forma pura en el origen (0,0)
cx_raw = (np.min(x_raw_cont) + np.max(x_raw_cont)) / 2
cy_raw = (np.min(y_raw_cont) + np.max(y_raw_cont)) / 2

x_cent_cont = x_raw_cont - cx_raw
y_cent_cont = y_raw_cont - cy_raw
x_cent_disc = x_raw_disc - cx_raw
y_cent_disc = y_raw_disc - cy_raw

# B) Aplicar Rotación en el origen
R = np.array([[np.cos(phi), -np.sin(phi)],
              [np.sin(phi),  np.cos(phi)]])

pts_rot0_cont = R @ np.vstack((x_cent_cont, y_cent_cont))
x_rot0_cont, y_rot0_cont = pts_rot0_cont[0, :], pts_rot0_cont[1, :]

pts_rot0_disc = R @ np.vstack((x_cent_disc, y_cent_disc))
x_rot0_disc, y_rot0_disc = pts_rot0_disc[0, :], pts_rot0_disc[1, :]

# C) CALCULAR EL FACTOR DE ESCALA SOBRE LA FORMA YA ROTADA (CORRECCIÓN)
x_min_rot, x_max_rot = np.min(x_rot0_cont), np.max(x_rot0_cont)
y_min_rot, y_max_rot = np.min(y_rot0_cont), np.max(y_rot0_cont)
escala = lado / max(x_max_rot - x_min_rot, y_max_rot - y_min_rot)

# D) Escalar uniformemente y trasladar a la mesa de trabajo
cx = x_start + lado / 2
cy = y_start + lado / 2

x_rot_cont = x_rot0_cont * escala + cx
y_rot_cont = y_rot0_cont * escala + cy

x_rot_disc = x_rot0_disc * escala + cx
y_rot_disc = y_rot0_disc * escala + cy

# ========================================================
# 3) CINEMÁTICA INVERSA
# ========================================================
def calc_inv_kinematics(x_arr, y_arr):
    q1 = np.zeros(len(x_arr))
    q2 = np.zeros(len(x_arr))
    elbow = "down"
    for k in range(len(x_arr)):
        px, py = x_arr[k], y_arr[k]
        cos_q2 = (px**2 + py**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_q2 = max(min(cos_q2, 1), -1)
        
        sin_abs = np.sqrt(max(0, 1 - cos_q2**2))
        sin_cand = sin_abs if elbow == "down" else -sin_abs
        
        q2_a = np.arctan2(sin_cand, cos_q2)
        q2_b = np.arctan2(-sin_cand, cos_q2)
        
        q1_a = np.arctan2(py, px) - np.arctan2(L2 * np.sin(q2_a), L1 + L2 * np.cos(q2_a))
        q1_b = np.arctan2(py, px) - np.arctan2(L2 * np.sin(q2_b), L1 + L2 * np.cos(q2_b))
        
        if k == 0:
            q2[k], q1[k] = q2_a, q1_a
        else:
            def wrap_to_pi(angle): return (angle + np.pi) % (2 * np.pi) - np.pi
            da = abs(wrap_to_pi(q2_a - q2[k-1])) + abs(wrap_to_pi(q1_a - q1[k-1]))
            db = abs(wrap_to_pi(q2_b - q2[k-1])) + abs(wrap_to_pi(q1_b - q1[k-1]))
            if da <= db:
                q2[k], q1[k] = q2_a, q1_a
            else:
                q2[k], q1[k] = q2_b, q1_b
    return np.unwrap(q1), np.unwrap(q2)

q1_disc, q2_disc = calc_inv_kinematics(x_rot_disc, y_rot_disc)

# ========================================================
# 4) VECTORES DE APROXIMACIÓN (EMPAME SPLINE C^2)
# ========================================================
dx = np.diff(x_rot_disc)
dy = np.diff(y_rot_disc)
ds = np.sqrt(dx**2 + dy**2)
ds[ds < 1e-9] = 1e-9
t_acum = np.zeros(N_clover)
t_acum[1:] = np.cumsum(ds) / v_cart

# Al cambiar la rotación y escala, q1_disc[0] y q2_disc[0] cambian automáticamente. 
# La spline siempre empalmará perfecta al nuevo punto inicial.
coef1 = solve_spline(-3, -np.pi/2, 0, q1_disc[0], t_acum[2], q1_disc[2])
coef2 = solve_spline(-3, 0, 0, q2_disc[0], t_acum[2], q2_disc[2])

xx_app_disc = np.linspace(-3, 0, N_approach)
q1_app_std_disc = coef1[0]*xx_app_disc**3 + coef1[1]*xx_app_disc**2 + coef1[2]*xx_app_disc + coef1[3]
q2_app_std_disc = coef2[0]*xx_app_disc**3 + coef2[1]*xx_app_disc**2 + coef2[2]*xx_app_disc + coef2[3]

xx_app_cont = np.linspace(-3, 0, N_cont)
q1_app_std_cont = coef1[0]*xx_app_cont**3 + coef1[1]*xx_app_cont**2 + coef1[2]*xx_app_cont + coef1[3]
q2_app_std_cont = coef2[0]*xx_app_cont**3 + coef2[1]*xx_app_cont**2 + coef2[2]*xx_app_cont + coef2[3]

x_app_cont = L1 * np.cos(q1_app_std_cont) + L2 * np.cos(q1_app_std_cont + q2_app_std_cont)
y_app_cont = L1 * np.sin(q1_app_std_cont) + L2 * np.sin(q1_app_std_cont + q2_app_std_cont)

x_app_disc = L1 * np.cos(q1_app_std_disc) + L2 * np.cos(q1_app_std_disc + q2_app_std_disc)
y_app_disc = L1 * np.sin(q1_app_std_disc) + L2 * np.sin(q1_app_std_disc + q2_app_std_disc)

# ========================================================
# 5) EXTRACCIÓN DE LOS 4 VECTORES (Grados Mecánicos)
# ========================================================
vec1_q1_app_deg = np.degrees(q1_app_std_disc + np.pi/2)
vec2_q2_app_deg = np.degrees(q2_app_std_disc)
vec3_q1_clv_deg = np.degrees(q1_disc + np.pi/2)
vec4_q2_clv_deg = np.degrees(q2_disc)

print("//////////////////////////////////////////////")
print("//////////////////////////////////////////////")
joint_1 = np.hstack((vec1_q1_app_deg,vec3_q1_clv_deg))
for angle in joint_1:
    print(round(angle,2),end=',')
print()
print("//////////////////////////////////////////////")
print("//////////////////////////////////////////////")
joint_2 = np.hstack((vec2_q2_app_deg,vec4_q2_clv_deg))
for angle in joint_2:
    print(round(angle,2),end=',')
print()

# ========================================================
# 6) GRÁFICAS
# ========================================================
plt.figure(figsize=(11, 9))

plt.subplot(2, 1, 1)
plt.plot(x_rot_cont*100, y_rot_cont*100, 'b-', linewidth=2, label='Trébol Continuo')
plt.plot(x_app_cont*100, y_app_cont*100, 'g-', linewidth=2, label='Aproximación Continua')

plt.plot(x_rot_disc*100, y_rot_disc*100, 'ro', markersize=3, label=f'Trébol (N={N_clover})')
plt.plot(x_app_disc*100, y_app_disc*100, 'mo', markersize=3, label=f'Aprox (N={N_approach})')

x_cuad = [x_start, x_start+lado, x_start+lado, x_start, x_start]
y_cuad = [y_start, y_start, y_start+lado, y_start+lado, y_start]
plt.plot(np.array(x_cuad)*100, np.array(y_cuad)*100, 'c--', label='Cuadrado (Límite Constante)')

plt.title(f'Espacio Cartesiano (Rotación: {phi_deg}°, Lado: {lado*100}cm)')
plt.xlabel('X (cm)')
plt.ylabel('Y (cm)')
plt.axis('equal')
plt.grid(True)
plt.legend(loc='lower left', fontsize=8)

plt.subplot(2, 1, 2)
idx_app = np.arange(-N_approach, 0)
idx_clv = np.arange(0, N_clover)

plt.plot(idx_app, vec1_q1_app_deg, 'b--', linewidth=2, label='Q1 Aproximación (Mecánico)')
plt.plot(idx_app, vec2_q2_app_deg, 'r--', linewidth=2, label='Q2 Aproximación (Mecánico)')
plt.plot(idx_clv, vec3_q1_clv_deg, 'b-', linewidth=2, label='Q1 Trébol (Mecánico)')
plt.plot(idx_clv, vec4_q2_clv_deg, 'r-', linewidth=2, label='Q2 Trébol (Mecánico)')

plt.axvline(x=0, color='k', linestyle=':', label='Empalme (x=0)')
plt.title('Articulaciones (Continuidad Garantizada por la Spline)')
plt.xlabel('Índice Discreto')
plt.ylabel('Ángulo (Grados)')
plt.grid(True)
plt.legend(fontsize=8)

plt.tight_layout()
plt.show()