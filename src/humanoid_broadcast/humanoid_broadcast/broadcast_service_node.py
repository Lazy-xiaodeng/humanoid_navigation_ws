#!/usr/bin/env python3
import json
import os
import shlex
import subprocess
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node

from humanoid_interfaces.srv import (
    GetBroadcastHealth,
    PlayBroadcast,
    SetBroadcastVolume,
)


@dataclass
class AudioDevice:
    backend: str
    name: str
    description: str
    alsa_device: str = ""


class SpeakerSelector:
    def __init__(self, logger):
        self.logger = logger

    def select(self) -> AudioDevice:
        forced_sink = os.getenv("XIAORUI_AUDIO_SINK", "").strip()
        forced_alsa = os.getenv("XIAORUI_ALSA_DEVICE", "").strip()
        forced_backend = os.getenv("XIAORUI_AUDIO_BACKEND", "auto").strip().lower()

        if forced_backend in {"alsa", "auto"} and forced_alsa and forced_alsa.lower() != "auto":
            return AudioDevice("alsa", forced_alsa, f"forced ALSA device {forced_alsa}", forced_alsa)

        if forced_backend in {"pipewire", "pulse", "auto"}:
            sinks = self._list_pulse_sinks()
            if forced_sink and forced_sink.lower() != "auto":
                match = next((sink for sink in sinks if forced_sink in sink.name or forced_sink in sink.description), None)
                return match or AudioDevice("pipewire", forced_sink, f"forced sink {forced_sink}")

            default_sink = self._pulse_default_sink()
            if default_sink:
                match = next((sink for sink in sinks if sink.name == default_sink), None)
                if match:
                    return match

            preferred = self._choose_preferred_sink(sinks)
            if preferred:
                return preferred

        alsa_devices = self._list_alsa_devices()
        if alsa_devices:
            return self._choose_preferred_alsa(alsa_devices)

        return AudioDevice("none", "", "no playback device detected")

    def set_volume(self, device: AudioDevice, volume_percent: int) -> None:
        volume = max(0, min(100, int(volume_percent)))
        if device.backend in {"pipewire", "pulse"} and device.name:
            self._run(["pactl", "set-sink-volume", device.name, f"{volume}%"], check=True)
            return
        if device.backend == "alsa" and device.alsa_device:
            self._run(["amixer", "-D", device.alsa_device, "sset", "Master", f"{volume}%"], check=False)

    def _list_pulse_sinks(self) -> List[AudioDevice]:
        result = self._run(["pactl", "list", "short", "sinks"], check=False)
        if result.returncode != 0:
            return []
        sinks: List[AudioDevice] = []
        for line in result.stdout.splitlines():
            parts = line.split("\t")
            if len(parts) < 2:
                continue
            name = parts[1].strip()
            description = " ".join(parts[1:]).strip()
            sinks.append(AudioDevice("pipewire", name, description))
        return sinks

    def _pulse_default_sink(self) -> str:
        result = self._run(["pactl", "get-default-sink"], check=False)
        return result.stdout.strip() if result.returncode == 0 else ""

    def _choose_preferred_sink(self, sinks: List[AudioDevice]) -> Optional[AudioDevice]:
        if not sinks:
            return None
        scored = sorted(sinks, key=lambda sink: self._score_audio_name(sink.description), reverse=True)
        return scored[0]

    def _list_alsa_devices(self) -> List[AudioDevice]:
        result = self._run(["aplay", "-l"], check=False)
        if result.returncode != 0:
            return []
        devices: List[AudioDevice] = []
        current_card = ""
        for line in result.stdout.splitlines():
            stripped = line.strip()
            if not stripped.startswith("card ") or "device " not in stripped:
                continue
            try:
                card_part, device_part = stripped.split("device ", 1)
                card_id = card_part.split(":", 1)[0].replace("card", "").strip()
                device_id = device_part.split(":", 1)[0].strip()
                current_card = card_id
                name = f"plughw:{card_id},{device_id}"
                devices.append(AudioDevice("alsa", name, stripped, name))
            except Exception:
                if current_card:
                    self.logger.debug(f"ignored ALSA line for card {current_card}: {stripped}")
        return devices

    def _choose_preferred_alsa(self, devices: List[AudioDevice]) -> AudioDevice:
        return sorted(devices, key=lambda device: self._score_audio_name(device.description), reverse=True)[0]

    @staticmethod
    def _score_audio_name(value: str) -> int:
        text = value.lower()
        score = 0
        if "usb" in text:
            score += 50
        if "analog" in text or "模拟" in text:
            score += 30
        if "speaker" in text or "audio" in text or "声" in text:
            score += 10
        if "hdmi" in text:
            score -= 40
        return score

    @staticmethod
    def _run(command: List[str], check: bool) -> subprocess.CompletedProcess:
        return subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=check)


class BroadcastPlayer:
    def __init__(self, logger):
        self.logger = logger

    def play(self, text: str, volume_percent: int, device: AudioDevice, broadcast_id: str, waypoint_id: str) -> str:
        mode = os.getenv("XIAORUI_BROADCAST_PLAYER", "dry_run").strip().lower()
        if mode in {"dry_run", "none", ""}:
            time.sleep(float(os.getenv("XIAORUI_BROADCAST_DRY_RUN_SEC", "0.2")))
            return "dry-run playback completed"

        command_template = os.getenv("XIAORUI_BROADCAST_PLAYER_COMMAND", "").strip()
        if mode == "command" and command_template:
            self._run_external_command(command_template, text, volume_percent, device, broadcast_id, waypoint_id)
            return "external playback command completed"

        if mode == "beep":
            self._play_beep(device)
            return "test beep completed"

        raise RuntimeError(f"unsupported broadcast player mode: {mode}")

    def _run_external_command(
        self,
        command_template: str,
        text: str,
        volume_percent: int,
        device: AudioDevice,
        broadcast_id: str,
        waypoint_id: str,
    ) -> None:
        command = command_template
        replacements = {
            "{{text}}": text,
            "{{volumePercent}}": str(volume_percent),
            "{{broadcastId}}": broadcast_id,
            "{{waypointId}}": waypoint_id,
            "{{selectedDevice}}": device.name,
            "{{backend}}": device.backend,
        }
        for key, value in replacements.items():
            command = command.replace(key, value)
        result = subprocess.run(
            command,
            shell=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env={
                **os.environ,
                "XIAORUI_BROADCAST_TEXT": text,
                "XIAORUI_BROADCAST_VOLUME": str(volume_percent),
                "XIAORUI_BROADCAST_ID": broadcast_id,
                "XIAORUI_BROADCAST_WAYPOINT_ID": waypoint_id,
                "XIAORUI_AUDIO_SELECTED_DEVICE": device.name,
                "XIAORUI_AUDIO_BACKEND_SELECTED": device.backend,
            },
        )
        if result.returncode != 0:
            reason = result.stderr.strip() or result.stdout.strip() or f"exit code {result.returncode}"
            raise RuntimeError(reason)

    def _play_beep(self, device: AudioDevice) -> None:
        with tempfile.TemporaryDirectory(prefix="xiaorui_broadcast_") as tmpdir:
            wav_path = Path(tmpdir) / "test.wav"
            subprocess.run(
                [
                    "python3",
                    "-c",
                    (
                        "import math, wave, struct; "
                        "p='{}'; rate=16000; dur=0.35; "
                        "w=wave.open(p,'w'); w.setnchannels(1); w.setsampwidth(2); w.setframerate(rate); "
                        "w.writeframes(b''.join(struct.pack('<h', int(12000*math.sin(2*math.pi*880*i/rate))) "
                        "for i in range(int(rate*dur)))); w.close()"
                    ).format(str(wav_path)),
                ],
                check=True,
            )
            if device.backend == "alsa" and device.alsa_device:
                subprocess.run(["aplay", "-D", device.alsa_device, str(wav_path)], check=True)
            else:
                subprocess.run(["ffplay", "-nodisp", "-autoexit", "-loglevel", "error", str(wav_path)], check=True)


class BroadcastServiceNode(Node):
    def __init__(self):
        super().__init__("xiaorui_broadcast_service")
        self.selector = SpeakerSelector(self.get_logger())
        self.player = BroadcastPlayer(self.get_logger())
        self.current_volume_percent = int(os.getenv("XIAORUI_BROADCAST_DEFAULT_VOLUME", "72"))
        self.selected_device = self.selector.select()

        self.play_srv = self.create_service(PlayBroadcast, "/xiaorui_broadcast/play", self.handle_play)
        self.set_volume_srv = self.create_service(SetBroadcastVolume, "/xiaorui_broadcast/set_volume", self.handle_set_volume)
        self.health_srv = self.create_service(GetBroadcastHealth, "/xiaorui_broadcast/health", self.handle_health)

        self.get_logger().info(
            "broadcast service ready: "
            f"backend={self.selected_device.backend}, device={self.selected_device.description}"
        )

    def handle_play(self, request: PlayBroadcast.Request, response: PlayBroadcast.Response):
        started_at = time.time()
        volume = self.normalize_volume(
            request.volume_percent if request.use_request_volume else self.current_volume_percent
        )
        try:
            self.refresh_device_if_needed()
            self.selector.set_volume(self.selected_device, volume)
            message = self.player.play(
                request.text,
                volume,
                self.selected_device,
                request.broadcast_id,
                request.waypoint_id,
            )
            response.success = True
            response.message = message
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        response.duration_sec = max(0.0, time.time() - started_at)
        response.selected_device = self.selected_device.description
        response.backend = self.selected_device.backend
        return response

    def handle_set_volume(self, request: SetBroadcastVolume.Request, response: SetBroadcastVolume.Response):
        volume = self.normalize_volume(request.volume_percent)
        try:
            self.refresh_device_if_needed()
            self.selector.set_volume(self.selected_device, volume)
            self.current_volume_percent = volume
            response.success = True
            response.message = "broadcast volume applied"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        response.applied_volume_percent = self.current_volume_percent
        response.selected_device = self.selected_device.description
        response.backend = self.selected_device.backend
        return response

    def handle_health(self, request: GetBroadcastHealth.Request, response: GetBroadcastHealth.Response):
        del request
        self.refresh_device_if_needed()
        response.ready = self.selected_device.backend != "none"
        response.selected_device = self.selected_device.description
        response.backend = self.selected_device.backend
        response.message = "ready" if response.ready else "no playback device detected"
        response.current_volume_percent = self.current_volume_percent
        return response

    def refresh_device_if_needed(self) -> None:
        if self.selected_device.backend == "none" or os.getenv("XIAORUI_AUDIO_RESELECT_EACH_REQUEST", "false").lower() == "true":
            self.selected_device = self.selector.select()

    @staticmethod
    def normalize_volume(value: int) -> int:
        return max(0, min(100, int(value)))


def main(args=None):
    rclpy.init(args=args)
    node = BroadcastServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
