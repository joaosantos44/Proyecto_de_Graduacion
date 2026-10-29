"""
robotat_tracker.py — Trackea la pose (x, y, theta) de uno o varios agentes
del Robotat via MQTT, reutilizando MqttRobotat (buffer circular) por cada
agent_id de interes.

Usa el topico 'mocap/all', que trae TODOS los rigidos que trackea OptiTrack
(robots del swarm Y marcadores sueltos, como el x usado como meta) -- no
solo los robots como 'mocap/swarm'. El formato de este topico es un poco
distinto: el id viene en el campo "identifier" (string, no int) y el
payload viene en "payload" (no "pld"). Se soportan ambos formatos por si
en algun punto se necesita volver a 'mocap/swarm'.

A diferencia de mqtt_cb.py (que filtra UN solo agent_id), aca se necesita
seguir varios agentes a la vez (el propio + el lider/marcador seguido),
asi que el callback despacha el paquete al manager correcto segun el id.
"""
import json
import time
from typing import Dict, List, Optional, Tuple

import paho.mqtt.client as mqtt
from scipy.spatial.transform import Rotation

from mqtt_robotat import MqttRobotat

ROBOTAT_HOST = '192.168.50.200'
ROBOTAT_PORT = 1880
ROBOTAT_TOPIC = 'mocap/all'


class RobotatTracker:
    def __init__(self, agent_ids: List[int], host: str = ROBOTAT_HOST,
                 port: int = ROBOTAT_PORT, topic: str = ROBOTAT_TOPIC):
        self.agent_ids = list(agent_ids)
        self.managers: Dict[int, MqttRobotat] = {
            aid: MqttRobotat(max_history=100) for aid in self.agent_ids
        }
        self.host = host
        self.port = port
        self.topic = topic

        self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self._client.on_message = self._on_message

    # ---- MQTT plumbing ----

    @staticmethod
    def _extract_id(data) -> Optional[int]:
        """Soporta 'identifier' (mocap/all, string) y 'pid' (mocap/swarm, int)."""
        if not isinstance(data, dict):
            return None
        raw = data.get('identifier', data.get('pid'))
        if raw is None:
            return None
        try:
            return int(raw)
        except (TypeError, ValueError):
            return None

    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return

        pid = self._extract_id(data)
        manager = self.managers.get(pid)
        if manager is not None:
            manager.update_latest(data)
            manager.append_latest_to_history()

    def connect(self, keepalive: int = 180):
        self._client.connect(self.host, port=self.port, keepalive=keepalive)
        self._client.subscribe(self.topic)
        self._client.loop_start()  # procesa mensajes en un hilo de fondo

    def disconnect(self):
        self._client.loop_stop()
        self._client.disconnect()

    # ---- Lectura de pose ----

    @staticmethod
    def _pose_from_packet(data) -> Optional[Tuple[float, float, float]]:
        """Extrae (x_mm, y_mm, theta_rad) de un paquete crudo del Robotat.
        Soporta tanto 'payload' (mocap/all) como 'pld' (mocap/swarm)."""
        try:
            payload = data.get('payload', data.get('pld'))
            pos = payload['pose']['position']
            rot = payload['pose']['rotation']
            x_mm = float(pos['x']) * 1000.0
            y_mm = float(pos['y']) * 1000.0
            qx, qy, qz, qw = float(rot['qx']), float(rot['qy']), float(rot['qz']), float(rot['qw'])
        except (KeyError, TypeError, AttributeError):
            return None

        # yaw = primer elemento de la secuencia ZYX (equivalente a
        # quat2eul(q,'ZYX') de MATLAB, ver Pruebas_obtencion_datos.m)
        eul = Rotation.from_quat([qx, qy, qz, qw]).as_euler('ZYX', degrees=False)
        theta_rad = float(eul[0])
        return x_mm, y_mm, theta_rad

    def try_get_pose(self, agent_id: int) -> Optional[Tuple[float, float, float]]:
        """Devuelve (x_mm, y_mm, theta_rad) sin bloquear, o None si aun no hay dato."""
        manager = self.managers[agent_id]
        d = manager.get_latest()
        if d is None:
            return None
        return self._pose_from_packet(d)

    def wait_for_pose(self, agent_id: int, timeout: float = 15.0,
                       poll_interval: float = 0.05) -> Tuple[float, float, float]:
        """Bloquea hasta obtener una pose valida de agent_id, o lanza TimeoutError."""
        t0 = time.monotonic()
        while True:
            pose = self.try_get_pose(agent_id)
            if pose is not None:
                return pose
            if time.monotonic() - t0 > timeout:
                raise TimeoutError(
                    f'No se recibio pose valida para agent_id={agent_id} en {timeout}s. '
                    f'Verifica que el marcador este siendo trackeado en el Robotat.'
                )
            time.sleep(poll_interval)
