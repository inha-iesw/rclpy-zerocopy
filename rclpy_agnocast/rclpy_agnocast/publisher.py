"""Publisher and Loan ownership wrappers."""

from __future__ import annotations

import warnings
from typing import Any, Optional, Type

from . import _native


class Loan:
    """One native SharedBuffer loan, valid through one publish or cancel."""

    __slots__ = ("_publisher", "_state")

    def __init__(self, publisher: "Publisher", state: _native.LoanState) -> None:
        self._publisher = publisher
        self._state = state

    @property
    def sequence(self) -> int:
        """Return the scalar sequence while this Loan is filling."""
        return self._state.sequence

    @sequence.setter
    def sequence(self, value: int) -> None:
        self._state.sequence = value

    @property
    def buffer(self) -> memoryview:
        """Create a fresh writable view whose aliases block terminal operations."""
        return self._state.buffer

    @property
    def phase(self) -> _native.LoanPhase:
        """Return the native lifecycle phase."""
        return self._state.phase

    def publish(self) -> None:
        """Publish through the Loan's owning Publisher."""
        self._publisher.publish(self)

    def cancel(self) -> None:
        """Cancel through the Loan's owning Publisher."""
        self._publisher.cancel(self)

    def __enter__(self) -> "Loan":
        return self

    def __exit__(
        self,
        exception_type: Optional[Type[BaseException]],
        exception: Optional[BaseException],
        traceback: Any,
    ) -> bool:
        del exception, traceback
        if self.phase in (_native.LoanPhase.PUBLISHED, _native.LoanPhase.CANCELLED):
            return False
        if exception_type is None:
            self.cancel()
        else:
            try:
                self._publisher._cancel(self, defer_if_exported=True)
            except BaseException:
                # Cleanup must never replace the exception raised by the context body.
                pass
        return False

    def __del__(self) -> None:
        state = getattr(self, "_state", None)
        if state is None:
            return
        try:
            if state.phase in (_native.LoanPhase.FILLING, _native.LoanPhase.CANCEL_PENDING):
                warnings.warn(
                    "unpublished Agnocast Loan was abandoned; cancellation was queued",
                    ResourceWarning,
                    source=self,
                )
            state.abandon()
        except BaseException:
            pass


class Publisher:
    """Typed SharedBuffer publisher backed by the process-global native worker."""

    __slots__ = ("_closed", "_handle", "_runtime_owner")

    def __init__(
        self, handle: _native.PublisherHandle, *, runtime_owner: Any = None
    ) -> None:
        self._handle = handle
        self._runtime_owner = runtime_owner
        self._closed = False

    @property
    def max_payload_bytes(self) -> int:
        return self._handle.max_payload_bytes

    def borrow(self, nbytes: int) -> Loan:
        """Borrow exactly ``nbytes`` destination bytes without a source copy."""
        if self._closed:
            raise RuntimeError("publisher is closed")
        if isinstance(nbytes, bool) or not isinstance(nbytes, int):
            raise TypeError("nbytes must be an integer")
        if nbytes < 0:
            raise ValueError("nbytes must be non-negative")
        return Loan(self, self._handle.borrow(nbytes))

    def publish(self, loan: Loan) -> None:
        """Commit one Loan after all writable aliases have been released."""
        self._validate_loan(loan)
        self._handle.publish(loan._state)

    def cancel(self, loan: Loan) -> None:
        """Cancel one Loan; active writable aliases make this fail safely."""
        self._cancel(loan, defer_if_exported=False)

    def _cancel(self, loan: Loan, *, defer_if_exported: bool) -> None:
        self._validate_loan(loan)
        self._handle.cancel(loan._state, defer_if_exported)

    @staticmethod
    def _validate_loan(loan: Loan) -> None:
        if not isinstance(loan, Loan):
            raise TypeError("loan must be a Loan")

    def close(self) -> None:
        """Stop admission and destroy the idle endpoint; retry after active Loan cleanup."""
        if self._closed:
            return
        self._handle.close()
        self._closed = True
        self._runtime_owner = None

    def __del__(self) -> None:
        if getattr(self, "_closed", True):
            return
        try:
            self.close()
        except BaseException:
            pass
