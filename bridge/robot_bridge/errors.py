class ArmInputRejected(RuntimeError):
    """A request cannot be executed; this alone is not a controller emergency."""


class ArmNotReady(ArmInputRejected):
    """Feedback, mode or enable status does not permit motion."""
