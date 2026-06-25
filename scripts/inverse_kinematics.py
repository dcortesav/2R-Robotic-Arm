import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import threading
import queue
import time

# ========================================================
# CONFIGURACIÓN — ajustar antes de ejecutar
# ========================================================
SERIAL_PORT            = 'COM3'   # Linux: '/dev/ttyUSB0'  Mac: '/dev/cu.usbserial-...'
BAUD_RATE              = 115200
STEP_INTERVAL_MS       = 20       # ms por punto de trayectoria
NUM_CLOVER_REPETITIONS = 3        # repeticiones del trébol (aproximación no se repite)

# ========================================================
# 1) PARÁMETROS CONFIGURABLES (preservados)
# ========================================================
lado_base = 20e-2
factor_escalamiento = 1.2
lado    = lado_base * factor_escalamiento
phi_deg = 0.0

N_clover   = 400
N_approach = 50

x_start = 5e-2
y_start = 0
epsilon   = 0.2
n_curvas  = 7
v_cart    = 10e-2

phi = np.radians(phi_deg)

L1 = 0.24
L2 = 0.19

# ========================================================
# 2) GENERACIÓN GEOMÉTRICA (preservada)
# ========================================================
N_cont = 1000
theta_cont = np.linspace(0, 2 * np.pi, N_cont)
abs_suave_cont = np.sqrt(np.cos((n_curvas / 2) * theta_cont) ** 2 + epsilon ** 2)
x_raw_cont = (0.99 + abs_suave_cont) * np.cos(theta_cont)
y_raw_cont = (0.99 + abs_suave_cont) * np.sin(theta_cont)

theta_disc = np.linspace(0, 2 * np.pi, N_clover)
abs_suave_disc = np.sqrt(np.cos((n_curvas / 2) * theta_disc) ** 2 + epsilon ** 2)
x_raw_disc = (0.99 + abs_suave_disc) * np.cos(theta_disc)
y_raw_disc = (0.99 + abs_suave_disc) * np.sin(theta_disc)

cx_raw = (np.min(x_raw_cont) + np.max(x_raw_cont)) / 2
cy_raw = (np.min(y_raw_cont) + np.max(y_raw_cont)) / 2

x_cent_cont = x_raw_cont - cx_raw
y_cent_cont = y_raw_cont - cy_raw
x_cent_disc = x_raw_disc - cx_raw
y_cent_disc = y_raw_disc - cy_raw

R_rot = np.array([[np.cos(phi), -np.sin(phi)],
                  [np.sin(phi),  np.cos(phi)]])

pts_rot0_cont = R_rot @ np.vstack((x_cent_cont, y_cent_cont))
x_rot0_cont, y_rot0_cont = pts_rot0_cont[0, :], pts_rot0_cont[1, :]

pts_rot0_disc = R_rot @ np.vstack((x_cent_disc, y_cent_disc))
x_rot0_disc, y_rot0_disc = pts_rot0_disc[0, :], pts_rot0_disc[1, :]

x_min_rot, x_max_rot = np.min(x_rot0_cont), np.max(x_rot0_cont)
y_min_rot, y_max_rot = np.min(y_rot0_cont), np.max(y_rot0_cont)
escala = lado / max(x_max_rot - x_min_rot, y_max_rot - y_min_rot)

cx = x_start + lado / 2
cy = y_start + lado / 2

x_rot_cont = x_rot0_cont * escala + cx
y_rot_cont = y_rot0_cont * escala + cy

x_rot_disc = x_rot0_disc * escala + cx
y_rot_disc = y_rot0_disc * escala + cy

# ========================================================
# 3) CINEMÁTICA INVERSA (preservada)
# ========================================================
def calc_inv_kinematics(x_arr, y_arr):
    q1 = np.zeros(len(x_arr))
    q2 = np.zeros(len(x_arr))
    elbow = "down"
    for k in range(len(x_arr)):
        px, py = x_arr[k], y_arr[k]
        cos_q2 = (px ** 2 + py ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
        cos_q2 = max(min(cos_q2, 1), -1)
        sin_abs = np.sqrt(max(0, 1 - cos_q2 ** 2))
        sin_cand = sin_abs if elbow == "down" else -sin_abs
        q2_a = np.arctan2(sin_cand, cos_q2)
        q2_b = np.arctan2(-sin_cand, cos_q2)
        q1_a = np.arctan2(py, px) - np.arctan2(L2 * np.sin(q2_a), L1 + L2 * np.cos(q2_a))
        q1_b = np.arctan2(py, px) - np.arctan2(L2 * np.sin(q2_b), L1 + L2 * np.cos(q2_b))
        if k == 0:
            q2[k], q1[k] = q2_a, q1_a
        else:
            def wrap_to_pi(a): return (a + np.pi) % (2 * np.pi) - np.pi
            da = abs(wrap_to_pi(q2_a - q2[k - 1])) + abs(wrap_to_pi(q1_a - q1[k - 1]))
            db = abs(wrap_to_pi(q2_b - q2[k - 1])) + abs(wrap_to_pi(q1_b - q1[k - 1]))
            q2[k], q1[k] = (q2_a, q1_a) if da <= db else (q2_b, q1_b)
    return np.unwrap(q1), np.unwrap(q2)

q1_disc, q2_disc = calc_inv_kinematics(x_rot_disc, y_rot_disc)

# ========================================================
# 4) VECTORES DE APROXIMACIÓN (preservados)
# ========================================================
def solve_spline(x0, y0, x1, y1, x2, y2):
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
    return np.linalg.solve(A, b)

dx = np.diff(x_rot_disc)
dy = np.diff(y_rot_disc)
ds = np.sqrt(dx**2 + dy**2)
ds[ds < 1e-9] = 1e-9
t_acum = np.zeros(N_clover)
t_acum[1:] = np.cumsum(ds) / v_cart

coef1 = solve_spline(-3, -np.pi/2, 0, q1_disc[0], t_acum[2], q1_disc[2])
coef2 = solve_spline(-3, 0,        0, q2_disc[0], t_acum[2], q2_disc[2])

xx_app = np.linspace(-3, 0, N_approach)
q1_app = coef1[0]*xx_app**3 + coef1[1]*xx_app**2 + coef1[2]*xx_app + coef1[3]
q2_app = coef2[0]*xx_app**3 + coef2[1]*xx_app**2 + coef2[2]*xx_app + coef2[3]

xx_app_cont = np.linspace(-3, 0, N_cont)
q1_app_cont = coef1[0]*xx_app_cont**3 + coef1[1]*xx_app_cont**2 + coef1[2]*xx_app_cont + coef1[3]
q2_app_cont = coef2[0]*xx_app_cont**3 + coef2[1]*xx_app_cont**2 + coef2[2]*xx_app_cont + coef2[3]

x_app_cont = L1 * np.cos(q1_app_cont) + L2 * np.cos(q1_app_cont + q2_app_cont)
y_app_cont = L1 * np.sin(q1_app_cont) + L2 * np.sin(q1_app_cont + q2_app_cont)

x_app_disc = L1 * np.cos(q1_app) + L2 * np.cos(q1_app + q2_app)
y_app_disc = L1 * np.sin(q1_app) + L2 * np.sin(q1_app + q2_app)

# ========================================================
# 5) EXTRACCIÓN DE LOS 4 VECTORES (separados — sin hstack)
# ========================================================
# Motor 1: offset +90° (frame mecánico vs cinemático)
# Motor 2: sin offset
vec_m1_approach = np.degrees(q1_app   + np.pi / 2)   # articulación 1, aproximación
vec_m2_approach = np.degrees(q2_app)                  # articulación 2, aproximación
vec_m1_clover   = np.degrees(q1_disc  + np.pi / 2)   # articulación 1, trébol
vec_m2_clover   = np.degrees(q2_disc)                 # articulación 2, trébol

print(f"Trayectoria generada: approach={N_approach} pts, clover={N_clover} pts")

# ========================================================
# 6) CINEMÁTICA DIRECTA (para visualización)
# ========================================================
def forward_kinematics(th1_mech_deg, th2_mech_deg):
    """
    Convierte ángulos mecánicos del encoder a posición cartesiana.
    th1_mech = q1_kin + 90° → q1_kin = th1_mech - 90°
    th2_mech = q2_kin       → sin offset
    """
    q1 = np.radians(th1_mech_deg) - np.pi / 2
    q2 = np.radians(th2_mech_deg)
    x = L1 * np.cos(q1) + L2 * np.cos(q1 + q2)
    y = L1 * np.sin(q1) + L2 * np.sin(q1 + q2)
    return x * 100, y * 100  # cm

# ========================================================
# 7) PROTOCOLO SERIAL
# ========================================================
# Cola compartida entre el thread lector y el hilo principal
from_esp32 = queue.Queue()
_running   = True


def _serial_reader(ser: serial.Serial, q: queue.Queue) -> None:
    """Thread lector: lee líneas del puerto serial y las pone en la cola."""
    while _running:
        try:
            raw = ser.readline()
            if raw:
                line = raw.decode('utf-8', errors='ignore').strip()
                if line:
                    q.put(line)
        except Exception:
            break


def _wait_for(q: queue.Queue, expected: str, timeout: float = 30.0) -> None:
    """
    Bloquea hasta recibir una línea que empiece con `expected`.
    Imprime todo lo que llegue mientras espera.
    Lanza RuntimeError si vence el timeout.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            line = q.get(timeout=0.2)
            print(f"  [ESP32] {line}")
            if line.startswith('ERROR:'):
                raise RuntimeError(f"ESP32 reportó error: {line}")
            if line.startswith(expected):
                return
        except queue.Empty:
            pass
    raise TimeoutError(f"Timeout esperando '{expected}'")


def _send_array(ser: serial.Serial, q: queue.Queue,
                tag: str, arr: np.ndarray) -> None:
    """Envía un array de floats con prefijo `tag:`, espera ACK `OK:<tag>`."""
    csv = ','.join(f'{v:.4f}' for v in arr)
    msg = f'{tag}:{csv}\n'
    ser.write(msg.encode())
    _wait_for(q, f'OK:{tag}')


def send_trajectory(ser: serial.Serial, q: queue.Queue,
                    m1_app: np.ndarray, m2_app: np.ndarray,
                    m1_clv: np.ndarray, m2_clv: np.ndarray,
                    n_reps: int, step_ms: int) -> None:
    """
    Envía la trayectoria completa a la ESP32 siguiendo el protocolo:
      TRAJ_START → arrays → TRAJ_END → espera READY
    """
    print("\n── Enviando trayectoria ──────────────────────")

    # Cabecera
    header = f'TRAJ_START {len(m1_app)} {len(m1_clv)} {n_reps} {step_ms}\n'
    ser.write(header.encode())
    _wait_for(q, 'OK:HEADER')

    # Arrays (Python espera ACK antes de enviar el siguiente)
    for tag, arr in [('M1A', m1_app), ('M2A', m2_app),
                     ('M1C', m1_clv), ('M2C', m2_clv)]:
        print(f"  Enviando {tag} ({len(arr)} puntos)...", end=' ')
        _send_array(ser, q, tag, arr)
        print("OK")

    # Finalizar
    ser.write(b'TRAJ_END\n')
    _wait_for(q, 'READY')
    print("── ESP32 lista. Iniciando ejecución ──────────\n")

# ========================================================
# 8) VISUALIZACIÓN EN TIEMPO REAL
# ========================================================
real_xs: list = []
real_ys: list = []
_done = [False]


def _build_figure():
    """Construye la figura Matplotlib con trayectoria deseada pre-dibujada."""
    fig, ax = plt.subplots(figsize=(8, 8))

    # Trayectoria deseada (estática)
    ax.plot(x_app_cont * 100, y_app_cont * 100,
            'b--', lw=1.5, alpha=0.6, label='Aproximación deseada')
    ax.plot(x_rot_cont * 100, y_rot_cont * 100,
            'b-',  lw=2,   label='Trébol deseado')

    x_cuad = [x_start, x_start+lado, x_start+lado, x_start, x_start]
    y_cuad = [y_start, y_start, y_start+lado, y_start+lado, y_start]
    ax.plot(np.array(x_cuad)*100, np.array(y_cuad)*100,
            'c--', alpha=0.4, label='Área de trabajo')

    # Línea real (dinámica, actualizada por FuncAnimation)
    real_line, = ax.plot([], [], 'r-', lw=1.5, label='Trayectoria real (encoder)')

    ax.set_aspect('equal')
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_title(f'Brazo 2DOF — Trébol  |  {NUM_CLOVER_REPETITIONS} repeticiones')
    ax.grid(True, alpha=0.4)
    ax.legend(loc='lower left', fontsize=8)

    return fig, ax, real_line


def _update(frame, real_line, ax):
    """Función de actualización de FuncAnimation. Drena la cola y actualiza el plot."""
    while not from_esp32.empty():
        try:
            line = from_esp32.get_nowait()
        except queue.Empty:
            break

        if line.startswith('T:'):
            # T:<ms>,<th1>,<th2>,<sp1>,<sp2>
            parts = line[2:].split(',')
            if len(parts) == 5:
                try:
                    th1 = float(parts[1])
                    th2 = float(parts[2])
                    x, y = forward_kinematics(th1, th2)
                    real_xs.append(x)
                    real_ys.append(y)
                except ValueError:
                    pass
        elif line == 'DONE':
            _done[0] = True
            ax.set_title('Trayectoria completada ✓')

    if real_xs:
        real_line.set_data(real_xs, real_ys)

    if _done[0]:
        ani.event_source.stop()

    return (real_line,)

def _handshake(ser: serial.Serial, q: queue.Queue, timeout: float = 15.0) -> None:
    """
    Espera AWAITING_TRAJECTORY. Si no llega en 2s (ESP32 ya estaba corriendo),
    envía PING para provocar la respuesta.
    """
    deadline = time.time() + timeout
    ping_sent = False
    while time.time() < deadline:
        try:
            line = q.get(timeout=2.0)
            print(f"  [ESP32] {line}")
            if line == 'AWAITING_TRAJECTORY':
                return
        except queue.Empty:
            if not ping_sent:
                print("  Sin respuesta espontánea, enviando PING...")
                ser.write(b'PING\n')
                ping_sent = True
    raise TimeoutError("ESP32 no respondió al handshake")

# ========================================================
# 9) MAIN
# ========================================================
if __name__ == '__main__':
    # ── Conectar al puerto serial ──────────────────────
    print(f"Conectando a {SERIAL_PORT} @ {BAUD_RATE} baud...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

    # ── Iniciar thread lector ──────────────────────────
    reader = threading.Thread(target=_serial_reader, args=(ser, from_esp32), daemon=True)
    reader.start()
    time.sleep(0.1)

    # ── Esperar señal de arranque de la ESP32 ──────────
    print("Esperando señal de ESP32...")
    _handshake(ser, from_esp32, timeout=15.0)

    # ── Enviar trayectoria ─────────────────────────────
    send_trajectory(
        ser, from_esp32,
        vec_m1_approach, vec_m2_approach,
        vec_m1_clover,   vec_m2_clover,
        n_reps=NUM_CLOVER_REPETITIONS,
        step_ms=STEP_INTERVAL_MS,
    )

    # ── Configurar figura ──────────────────────────────
    fig, ax, real_line = _build_figure()

    # ── Animación en tiempo real ───────────────────────
    # interval=50ms → sincronizado con telemetría a 25 Hz
    ani = animation.FuncAnimation(
        fig,
        _update,
        fargs=(real_line, ax),
        interval=50,
        blit=True,
        cache_frame_data=False,
    )

    plt.tight_layout()
    plt.show()

    # ── Limpieza ───────────────────────────────────────
    _running = False
    ser.close()
    print("Puerto serial cerrado.")