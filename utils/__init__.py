"""utils — Frame capture, logging, video recording, terminal helpers."""
# No eager imports here — import directly from submodules to avoid
# triggering heavy dependencies (cv2, config.settings, etc.) whenever
# any utils submodule is imported.
__all__ = ["FrameGrabber", "VideoRecorder"]
