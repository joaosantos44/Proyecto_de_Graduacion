"""
mqtt_robotat.py — Guarda paquetes recibidos en un callback y permite
obtenerlos en el código principal.

Migrado de mqttRobotat.m

Uso:
    manager = MqttRobotat(max_history=300000)
    - manager.update_latest(data): guarda el último paquete
    - manager.append_latest_to_history(): añade el último paquete a un buffer circular
    - manager.get_latest(): devuelve el último paquete (dict) o None si no hay
    - manager.get_history_matrix(): devuelve el historial de pose (numpy array)
    - manager.clear_history(): limpia el historial
"""
import numpy as np


def _getfield_or_nan(d, key):
    """Helper: obtención segura de un campo de un dict."""
    if isinstance(d, dict) and key in d:
        try:
            return float(d[key])
        except (TypeError, ValueError):
            return float('nan')
    return float('nan')


class MqttRobotat:
    def __init__(self, max_history=300000):
        self.max_history = max_history
        self.count = 0     # numero de entradas validas actualmente
        self.head = -1     # indice del ultimo elemento insertado (0-based)

        self.xs = np.full(max_history, np.nan)
        self.ys = np.full(max_history, np.nan)
        self.zs = np.full(max_history, np.nan)
        self.qx = np.full(max_history, np.nan)
        self.qy = np.full(max_history, np.nan)
        self.qz = np.full(max_history, np.nan)
        self.qw = np.full(max_history, np.nan)

        self.latest = None  # ultimo paquete JSON decodificado (dict)

    def update_latest(self, data_dict):
        """Guarda el último paquete. data_dict suele venir de json.loads()."""
        self.latest = data_dict

    def append_latest_to_history(self):
        """Añade self.latest al buffer circular (si tiene posición)."""
        d = self.latest
        if d is None:
            print('append: Latest vacío')
            return False

        try:
            # Intentar extraer position & rotation con el path esperado:
            # payload["pld"]["pose"]["position"]["x"/"y"/"z"] (mocap/swarm) o
            # payload["payload"]["pose"]["position"]["x"/"y"/"z"] (mocap/all)
            # payload["pld"/"payload"]["pose"]["rotation"]["qx".."qw"]
            pld = d.get('pld', d.get('payload')) if isinstance(d, dict) else None
            pose = pld.get('pose') if isinstance(pld, dict) else None
            pos = pose.get('position') if isinstance(pose, dict) else None

            if pos is not None:
                rot = pose.get('rotation') if isinstance(pose, dict) else None

                idx = (self.head + 1) % self.max_history  # siguiente indice
                self.head = idx  # en que dato va (dentro del array)
                if self.count < self.max_history:
                    self.count += 1  # cuantos datos hay

                # guardar valores (formato float), si existen
                self.xs[idx] = _getfield_or_nan(pos, 'x')
                self.ys[idx] = _getfield_or_nan(pos, 'y')
                self.zs[idx] = _getfield_or_nan(pos, 'z')

                if rot is not None:
                    self.qx[idx] = _getfield_or_nan(rot, 'qx')
                    self.qy[idx] = _getfield_or_nan(rot, 'qy')
                    self.qz[idx] = _getfield_or_nan(rot, 'qz')
                    self.qw[idx] = _getfield_or_nan(rot, 'qw')
                else:
                    self.qx[idx] = np.nan
                    self.qy[idx] = np.nan
                    self.qz[idx] = np.nan
                    self.qw[idx] = np.nan

                return True
            else:
                print('append: estructura inesperada en Latest')
                return False
        except Exception as e:
            print(f'append: error al procesar paquete -> {e}')
            return False

    def get_latest(self):
        return self.latest

    def get_history_matrix(self):
        """
        Devuelve el historial como matriz numpy (n x 7) con columnas
        [x y z qx qy qz qw], en orden cronologico.
        """
        n = self.count
        if n == 0:
            return np.empty((0, 7))

        # Reconstruir indices circulares (0-based)
        idx_start = self.head - n + 1
        indices = (np.arange(idx_start, idx_start + n)) % self.max_history

        data = np.column_stack([
            self.xs[indices],
            self.ys[indices],
            self.zs[indices],
            self.qx[indices],
            self.qy[indices],
            self.qz[indices],
            self.qw[indices],
        ])
        return data

    def clear_history(self):
        self.count = 0
        self.head = -1
        self.xs[:] = np.nan
        self.ys[:] = np.nan
        self.zs[:] = np.nan
        self.qx[:] = np.nan
        self.qy[:] = np.nan
        self.qz[:] = np.nan
        self.qw[:] = np.nan
