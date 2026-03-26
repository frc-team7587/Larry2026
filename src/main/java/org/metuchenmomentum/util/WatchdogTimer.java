// Copyright 2026, Metuchen Momentum, FRC 7857
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.metuchenmomentum.util;

import java.util.concurrent.PriorityBlockingQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

/**
 * A watchdog timer that runs events after a deadline has expired. Timeouts can be canceled.
 *
 * <p>The implementation makes the following assumptions:
 *
 * <ul>
 *   <li>Some (unspecified) process invokes {@link WatchdogTimer#periodic periodic()}. on every
 *       timer tick.
 *   <li>The inter-tick time is uniform.
 *   <li>The robot will be rebooted before {@link Long#MAX_VALUE} {@code 2^63 -1}, roughly {@code
 *       9.2233720368548E+18} ticks. If the tick has its default 20 ms setting, that's {@code
 *       1.844674407×10¹⁷} seconds, which is a little mre than {@code 8093659099} years.
 * </ul>
 *
 * <p>Note that this class is thread-safe.
 */
public class WatchdogTimer {

  /**
   * Wraps around a {@link Runnable} to be invoked when its watchdog time expires. Provides an
   * absolute expiry time in ticks since construction, a {@link Runnable} implementation that
   * forwards to the user-provided {@link Runnable}, and an {@link AutoCloseable} implementation
   * that cancels the invocation.
   */
  public static final class PendingAction
      implements AutoCloseable, Comparable<PendingAction>, Runnable {

    /** Action to invoke on watchdog deadline expiry */
    private final Runnable onTimeout;
    /** Expiry time in ticks since construction. */
    private final long deadlineInPeriodicTicks;
    /**
     * Active flag. If {@code true}, the expiry action will run. If {@code false}, the action will
     * not run.
     */
    private final AtomicBoolean active;

    /**
     * Creates an instance from the specified arguments.
     *
     * @param onTimeout user-provided {@link Runnable} to be invoked on expiry
     * @param deadlineInPeriodicTicks the absolute expiry time in ticks since construction
     */
    private PendingAction(Runnable onTimeout, long deadlineInPeriodicTicks) {
      this.onTimeout = onTimeout;
      this.active = new AtomicBoolean(true);
      this.deadlineInPeriodicTicks = deadlineInPeriodicTicks;
    }

    /**
     * Orders {@link PendingAction} by expiry time.
     *
     * @param other the {@link PendingAction} to compare with {@code this}.
     * @return a negative integer, zero, or a positive integer as this object is less than, equal
     *     to, or greater than the specified object.
     * @see Comparable
     */
    @Override
    public int compareTo(PendingAction other) {
      return Long.compare(deadlineInPeriodicTicks, other.getDeadlineInPeriodicTicks());
    }

    /**
     * Closes this action, i.e. marks the enclose action as bypassed. Note that this method is
     * idempotent; multiple invocations have the same effect as one.
     *
     * @see AutoCloseable
     */
    @Override
    public void close() {
      active.set(false);
    }

    /**
     * @return the watchdog deadline in ticks since construction. Primarily used for testing.
     */
    long getDeadlineInPeriodicTicks() {
      return deadlineInPeriodicTicks;
    }

    /**
     * Runs the bound action if this {@link PendingAction} has not been closed; does nothing
     * otherwise.
     */
    @Override
    public void run() {
      if (active.get()) {
        onTimeout.run();
      }
    }
  }

  /**
   * The number of ticks since this instance was constructed, with each {@link #periodic()}
   * invocation counting as a single tick.
   */
  private final AtomicLong ticksSinceConstruction;

  private final PriorityBlockingQueue<PendingAction> actions;

  /** Creates an instance. Note that the */
  public WatchdogTimer() {
    ticksSinceConstruction = new AtomicLong();
    actions = new PriorityBlockingQueue<>();
  }

  /**
   * @return {@code true} if and only if no events are pending.
   */
  public boolean isEmpty() {
    return actions.isEmpty();
  }

  public void periodic() {
    var currentTime = ticksSinceConstruction.addAndGet(1);
    try {
      while (!actions.isEmpty() && actions.peek().getDeadlineInPeriodicTicks() <= currentTime) {
        actions.take().run();
      }
    } catch (InterruptedException e) {
      // TODO: log error. This should not happen.
    }
  }

  /**
   * Schedule the specified action to run after the specified deadline.
   *
   * @param action the action to run. Cannot be {@code null}. This is not checked. Note that the
   *     action will remain scheduled until its deadline expires, even if the user cancels it.
   *     However, a scheduled action will not run.
   * @param delayInTicks timeout in ticks relative to the current tick count. The value
   *     <em>SHOULD</em> be strictly positive. If it is zero or negative, the action will run at the
   *     next tick.
   * @return an {@link AutoCloseable} action handle. Users can invoke the handle's {@link
   *     AutoCloseable#close() close()} method to cancel the action. Note that cancelling the action
   *     <em>DOES NOT</em> will remain scheduled, but its action will not run when its deadline
   *     expires.
   */
  public AutoCloseable schedule(Runnable action, long delayInTicks) {
    var pendingAction = new PendingAction(action, ticksSinceConstruction.get() + delayInTicks);
    actions.add(pendingAction);
    return pendingAction;
  }

  /**
   * @return the number of pending actions. Note that the count <em>ALL</em> scheduled actions,
   *     including canceled actions.
   */
  public int size() {
    return actions.size();
  }
}
