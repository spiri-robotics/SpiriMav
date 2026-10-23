"""Shared fixtures: every test gets its own SpiriSynq session, off the network."""

from __future__ import annotations

import sys

import pytest
import zenoh
from loguru import logger
from SpiriSynq.session import Session, current_session


@pytest.fixture(autouse=True)
def loguru_to_current_stderr():
    """Logs to whatever stderr is when they're written.

    The CLI's callback adds a handler on the stderr of the moment, which under
    CliRunner is a capture stream closed after its test; later logs would hit it.
    """
    yield
    logger.remove()
    logger.add(lambda message: sys.stderr.write(message))


@pytest.fixture(autouse=True)
def synq_session():
    """A zenoh session with no scouting and no endpoints, as the current session."""
    conf = zenoh.Config()
    conf.insert_json5("scouting/multicast/enabled", "false")
    s = Session(config=conf)
    token = current_session.set(s)
    yield s
    current_session.reset(token)
    s.close()
