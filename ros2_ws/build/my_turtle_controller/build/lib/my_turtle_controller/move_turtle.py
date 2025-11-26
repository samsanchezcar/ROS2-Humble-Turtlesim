#!/usr/bin/env python3
"""
TurtleController mejorado para ROS2 + turtlesim.

Controles:
    Flechas     → Mover tortuga manualmente
    A, C, D, S  → Dibujar letras
    B           → Borrar pantalla
    P           → Levantar/bajar lápiz (toggle)
    R           → Reset de orientación (alinear a 0°)
    Q           → Salir
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import SetPen
from pynput import keyboard
import math
import time


class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        # Publisher
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Clientes de servicio
        self.clear_client = self.create_client(Empty, "/clear")
        self.pen_client = self.create_client(SetPen, "/turtle1/set_pen")

        # Estado interno
        self.theta = 0.0
        self.pen_is_up = False
        self.running = True

        # Velocidades
        self.lin_speed = 1.8
        self.ang_speed = 2.0

        # Espaciado entre letras
        self.letter_spacing = 0.5

        # Color del lápiz
        self.pen_color = (170, 130, 230)
        self.pen_width = 5

        self._print_help()

    def _print_help(self):
        self.get_logger().info("=" * 50)
        self.get_logger().info("TURTLE CONTROLLER - Controles:")
        self.get_logger().info("  Flechas    → Mover tortuga")
        self.get_logger().info("  A, C, D, S → Dibujar letras")
        self.get_logger().info("  B          → Borrar pantalla")
        self.get_logger().info("  P          → Toggle lápiz arriba/abajo")
        self.get_logger().info("  R          → Reset orientación (0°)")
        self.get_logger().info("  Q          → Salir")
        self.get_logger().info("=" * 50)

    # ===================================================================
    # MOVIMIENTO BÁSICO
    # ===================================================================
    def move(self, lin: float, ang: float, duration: float):
        """Mueve la tortuga con velocidad lineal y angular por una duración."""
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)

        t0 = time.time()
        while (time.time() - t0) < duration:
            self.publisher_.publish(msg)
            time.sleep(0.02)

        self.stop()

        # Actualizar ángulo estimado
        self.theta += ang * duration
        self.theta = self._wrap_angle(self.theta)

    def arc(self, linear: float, angular: float, duration: float):
        """Dibuja un arco con movimiento simultáneo lineal y angular."""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        t0 = time.time()
        while (time.time() - t0) < duration:
            self.publisher_.publish(msg)
            time.sleep(0.02)

        self.stop()

        # Actualizar ángulo
        self.theta += angular * duration
        self.theta = self._wrap_angle(self.theta)

    def stop(self):
        """Detiene la tortuga."""
        msg = Twist()
        self.publisher_.publish(msg)

    def _wrap_angle(self, angle: float) -> float:
        """Normaliza el ángulo al rango [-π, π]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    # ===================================================================
    # ALINEACIÓN GLOBAL
    # ===================================================================
    def align_to_angle(self, target_angle: float):
        """Alinea la tortuga a un ángulo absoluto."""
        diff = self._wrap_angle(target_angle - self.theta)

        if abs(diff) < 0.01:
            return

        ang = self.ang_speed if diff > 0 else -self.ang_speed
        duration = abs(diff / self.ang_speed)

        self.move(0.0, ang, duration)

    def align_horizontal(self):
        """Alinea la tortuga horizontalmente (0°)."""
        self.align_to_angle(0.0)

    # ===================================================================
    # CONTROL DEL LÁPIZ
    # ===================================================================
    def pen(self, up: bool):
        """Levanta o baja el lápiz."""
        if not self.pen_client.wait_for_service(timeout_sec=0.5):
            return

        req = SetPen.Request()
        req.r, req.g, req.b = self.pen_color
        req.width = self.pen_width
        req.off = 1 if up else 0

        self.pen_client.call_async(req)
        self.pen_is_up = up
        time.sleep(0.05)

    def toggle_pen(self):
        """Alterna el estado del lápiz."""
        self.pen(not self.pen_is_up)
        state = "ARRIBA" if self.pen_is_up else "ABAJO"
        self.get_logger().info(f"Lápiz: {state}")

    # ===================================================================
    # UTILIDADES
    # ===================================================================
    def clear_screen(self):
        """Borra los trazos de la pantalla."""
        if not self.clear_client.wait_for_service(timeout_sec=0.5):
            return

        self.clear_client.call_async(Empty.Request())
        self.get_logger().info("Pantalla borrada")

    def add_spacing(self):
        """Añade espacio horizontal después de una letra."""
        self.pen(True)
        self.align_horizontal()
        self.move(self.lin_speed, 0.0, self.letter_spacing / self.lin_speed)
        self.pen(False)

    def _prepare_letter(self):
        """Preparación común antes de dibujar cualquier letra."""
        self.pen(False)
        self.align_horizontal()

    # ===================================================================
    # DIBUJO DE LETRAS
    # ===================================================================

    def draw_A(self):
        """Letra A con geometría corregida."""
        self.get_logger().info("Dibujando A")
        self._prepare_letter()

        angle_deg = 68
        angle_rad = math.radians(angle_deg)
        leg_duration = 1.1

        # Calcular desplazamientos para posicionamiento preciso
        half_width = self.lin_speed * leg_duration * math.cos(angle_rad)

        # Diagonal izquierda (subiendo)
        self.align_to_angle(angle_rad)
        self.move(self.lin_speed, 0.0, leg_duration)

        # Diagonal derecha (bajando)
        self.align_to_angle(-angle_rad)
        self.move(self.lin_speed, 0.0, leg_duration)

        # Retroceder para trazo central (mitad de la diagonal derecha)
        self.move(-self.lin_speed, 0.0, leg_duration * 0.5)

        # Trazo horizontal central
        self.align_to_angle(math.pi)
        bar_width = half_width * 0.9
        self.move(self.lin_speed, 0.0, bar_width / self.lin_speed)

        # Volver a la base: ir hacia abajo-derecha
        self.pen(True)
        self.align_to_angle(-angle_rad)
        self.move(self.lin_speed, 0.0, leg_duration * 0.5)

        # Asegurar alineación horizontal al final
        self.align_horizontal()

        # Espacio después de la A
        self.move(self.lin_speed, 0.0, self.letter_spacing / self.lin_speed)
        self.add_spacing()
        self.pen(False)

        self.align_horizontal()


    def draw_C(self):
        """Letra C estilizada con curva suave (espejo de D)."""
        self.get_logger().info("Dibujando C")
        self._prepare_letter()

        # Mover al inicio (arriba a la derecha)
        self.pen(True)
        self.align_to_angle(math.pi / 2)
        self.move(self.lin_speed, 0.0, 1.0)
        self.align_horizontal()
        self.move(self.lin_speed, 0.0, 0.7)
        self.pen(False)

        # Parte superior horizontal corta (hacia la izquierda)
        self.align_to_angle(math.pi)
        self.move(self.lin_speed, 0.0, 0.15)

        # Arco grande hacia abajo (semicírculo izquierdo)
        self.arc(self.lin_speed, 1.8, 1.75)

        # Parte inferior horizontal corta (hacia la derecha)
        self.align_horizontal()
        self.move(self.lin_speed, 0.0, 0.15)

        # Posicionar para siguiente letra
        self.pen(True)
        self.align_horizontal()
        self.move(self.lin_speed, 0.0, 0.55)
        self.align_horizontal()
        self.pen(False)

    def draw_D(self):
        """Letra D con curva suave."""
        self.get_logger().info("Dibujando D")
        self._prepare_letter()

        # Línea vertical (subiendo)
        self.align_to_angle(math.pi / 2)
        self.move(self.lin_speed, 0.0, 1.0)

        # Parte superior horizontal corta
        self.align_horizontal()
        self.move(self.lin_speed, 0.0, 0.15)

        # Curva grande hacia abajo (semicírculo derecho)
        self.arc(self.lin_speed, -1.8, 1.75)

        # Cerrar en la base
        self.align_to_angle(math.pi)
        self.move(self.lin_speed, 0.0, 0.15)

        # Posicionar para siguiente letra
        self.pen(True)
        self.align_horizontal()
        self.move(self.lin_speed, 0.0, 0.85)
        self.align_horizontal()
        self.pen(False)

        self.add_spacing()

    def draw_S(self):
        """Letra S con curvas amplias, tamaño ajustado."""
        self.get_logger().info("Dibujando S")
        self._prepare_letter()

        # Velocidad lineal menor para curvas más anchas
        s_lin = 1
        # Velocidad angular menor para radio mayor
        s_ang = 2.3

        # Curva inferior (de izquierda a derecha, subiendo)
        self.align_to_angle(math.radians(-30))
        self.arc(s_lin, s_ang, 1)

        # Curva media (transición en S)
        self.arc(s_lin, s_ang, 0.5)

        # Curva media-alta (girando al otro lado)
        self.arc(s_lin, -s_ang, 0.5)

        # Curva superior (terminando hacia la izquierda)
        self.arc(s_lin, -s_ang, 1)

        # Posicionar para siguiente letra
        self.pen(True)
        self.align_to_angle(-math.pi / 2)
        self.move(self.lin_speed, 0.0, 1)
        self.align_horizontal()
        self.pen(False)

        self.add_spacing()

    # ===================================================================
    # CONTROL POR TECLADO
    # ===================================================================
    def on_key_press(self, key):
        """Callback al presionar una tecla."""
        msg = Twist()

        try:
            # Flechas
            if key == keyboard.Key.up:
                msg.linear.x = self.lin_speed
                self.publisher_.publish(msg)
            elif key == keyboard.Key.down:
                msg.linear.x = -self.lin_speed
                self.publisher_.publish(msg)
            elif key == keyboard.Key.left:
                msg.angular.z = self.ang_speed
                self.publisher_.publish(msg)
            elif key == keyboard.Key.right:
                msg.angular.z = -self.ang_speed
                self.publisher_.publish(msg)

            # Teclas con caracter
            elif hasattr(key, 'char') and key.char:
                char = key.char.lower()

                if char == 'a':
                    self.draw_A()
                elif char == 'c':
                    self.draw_C()
                elif char == 'd':
                    self.draw_D()
                elif char == 's':
                    self.draw_S()
                elif char == 'b':
                    self.clear_screen()
                elif char == 'p':
                    self.toggle_pen()
                elif char == 'r':
                    self.align_horizontal()
                    self.get_logger().info("Orientación reseteada a 0°")
                elif char == 'q':
                    self.get_logger().info("Saliendo...")
                    self.running = False

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def on_key_release(self, key):
        """Detiene movimiento al soltar flechas."""
        if key in [keyboard.Key.up, keyboard.Key.down,
                   keyboard.Key.left, keyboard.Key.right]:
            self.stop()

    # ===================================================================
    # LOOP PRINCIPAL
    # ===================================================================
    def run(self):
        """Loop principal con listener de teclado."""
        with keyboard.Listener(
            on_press=self.on_key_press,
            on_release=self.on_key_release
        ) as listener:
            while self.running:
                time.sleep(0.1)
            listener.stop()


# ===================================================================
# MAIN
# ===================================================================
def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()