# =========================================================================
# mqtt_cb.py — Función callback base. Desempaqueta el mensaje recibido
# e identifica el marcador de interés.
#   - topic: tópico donde se publican los paquetes.
#   - message: paquete recibido (bytes o str).
#   - agent_id: id del marcador de interés.
#   - manager: instancia de MqttRobotat que guarda el histórico (si se usa).
#   - scf: objeto del dron/agente conectado (si se usa; None si no aplica).
#
# Se debe agregar ÚNICAMENTE lo que se debe realizar cada vez que se recibe
# un paquete MQTT.
#
# Migrado de mqttCB.m
# =========================================================================
import json

# === NO MODIFICAR === #


def mqtt_cb(topic, message, agent_id, manager, scf):

    # == Variables (modificar) ===

    # === NO MODIFICAR ===
    try:
        # intentar convertir mensaje a string
        if isinstance(message, (bytes, bytearray)):
            msg_str = message.decode('utf-8')
        else:
            msg_str = str(message)

        try:
            data = json.loads(msg_str)  # intentar decodificar el paquete JSON
        except json.JSONDecodeError:
            print(f'Mensaje no es JSON válido: {msg_str}')
            return

        # Verificar si existe "pid" y coincide con el deseado
        if isinstance(data, dict) and data.get('pid') == agent_id:

            # === Lógica del callback (modificar) ===

            manager.update_latest(data)
            manager.append_latest_to_history()

    except Exception as e:
        print(f'Error en mqtt_cb: {e}')


def make_paho_on_message(agent_id, manager, scf=None):
    """
    Fábrica que envuelve mqtt_cb() con la firma que espera
    paho-mqtt para on_message: (client, userdata, msg).

    Uso:
        client.on_message = make_paho_on_message(agent_id, manager)
    """
    def _on_message(client, userdata, msg):
        mqtt_cb(msg.topic, msg.payload, agent_id, manager, scf)
    return _on_message
