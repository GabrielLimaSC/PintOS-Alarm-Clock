#include "devices/timer.h"
#include <debug.h>
#include <inttypes.h>
#include <round.h>
#include <stdio.h>
#include "devices/pit.h"
#include "threads/interrupt.h"
#include "threads/synch.h"
#include "threads/thread.h"

/* See [8254] for hardware details of the 8254 timer chip. */

#if TIMER_FREQ < 19
#error 8254 timer requires TIMER_FREQ >= 19
#endif
#if TIMER_FREQ > 1000
#error TIMER_FREQ <= 1000 recommended
#endif

/* Number of timer ticks since OS booted. */
static int64_t ticks;

/* Number of loops per timer tick.
   Initialized by timer_calibrate(). */
static unsigned loops_per_tick;

/* List of all processes who have to wait */
static struct list sleep_wait_list;

static intr_handler_func timer_interrupt;
static bool too_many_loops (unsigned loops);
static void busy_wait (int64_t loops);
static void real_time_sleep (int64_t num, int32_t denom);
static void real_time_delay (int64_t num, int32_t denom);

static struct thread *get_thread_from_elem(const struct list_elem *elem) {
  return list_entry(elem, struct thread, elem);
}

static void assert_non_null(const struct list_elem *elem) {
  ASSERT(elem != NULL);
}

static bool is_wake_time_less(const struct thread *a, const struct thread *b) {
  return a->wake_time < b->wake_time;
}

static bool compare_wake_times (const struct list_elem *a, const struct list_elem *b, void *aux UNUSED) {
  assert_non_null(a);
  assert_non_null(b);
  struct thread *thread_a = get_thread_from_elem(a);
  struct thread *thread_b = get_thread_from_elem(b);
  return is_wake_time_less(thread_a, thread_b);
}

static void insert_into_sleep_list(struct thread *cur_thread, int64_t wake_time) {
  enum intr_level old_level = intr_disable();
  cur_thread->wake_time = wake_time;
  list_insert_ordered(&sleep_wait_list, &cur_thread->elem, compare_wake_times, NULL);
  thread_block();
  intr_set_level(old_level);
}

static struct list_elem *get_next_elem(struct list_elem *e) {
  return list_next(e);
}

static struct thread *get_thread_from_list_elem(struct list_elem *e) {
  return list_entry(e, struct thread, elem);
}

static bool should_unblock_thread(struct thread *t, int64_t current_ticks) {
  return t->wake_time <= current_ticks;
}

static void unblock_thread_if_needed(struct thread *t) {
  thread_unblock(t);
}

static void iterate_and_unblock_threads(struct list_elem **e, int64_t current_ticks) {
  while (*e != list_end(&sleep_wait_list)) {
    struct list_elem *next_elem = get_next_elem(*e);
    struct thread *t = get_thread_from_list_elem(*e);

    if (!should_unblock_thread(t, current_ticks))
      break;

    list_remove(*e);
    unblock_thread_if_needed(t);
    *e = next_elem;
  }
}

static void wake_sleeping_threads() {
  struct list_elem *e = list_begin(&sleep_wait_list);
  int64_t current_ticks = timer_ticks();
  iterate_and_unblock_threads(&e, current_ticks);
}

/* Sets up the timer to interrupt TIMER_FREQ times per second,
   and registers the corresponding interrupt. */
void timer_init(void) {
  pit_configure_channel(0, 2, TIMER_FREQ);
  intr_register_ext(0x20, timer_interrupt, "8254 Timer");
  list_init(&sleep_wait_list);
}

/* Calibrates loops_per_tick, used to implement brief delays. */
void timer_calibrate(void) {
  unsigned high_bit, test_bit;

  ASSERT(intr_get_level() == INTR_ON);
  printf("Calibrating timer...  ");

  loops_per_tick = 1u << 10;
  while (!too_many_loops(loops_per_tick << 1)) {
    loops_per_tick <<= 1;
    ASSERT(loops_per_tick != 0);
  }

  high_bit = loops_per_tick;
  for (test_bit = high_bit >> 1; test_bit != high_bit >> 10; test_bit >>= 1)
    if (!too_many_loops(high_bit | test_bit))
      loops_per_tick |= test_bit;

  printf("%'"PRIu64" loops/s.\n", (uint64_t) loops_per_tick * TIMER_FREQ);
}

/* Returns the number of timer ticks since the OS booted. */
int64_t timer_ticks(void) {
  enum intr_level old_level = intr_disable();
  int64_t t = ticks;
  intr_set_level(old_level);
  return t;
}

/* Returns the number of timer ticks elapsed since THEN, which
   should be a value once returned by timer_ticks(). */
int64_t timer_elapsed(int64_t then) {
  return timer_ticks() - then;
}

/* Sleeps for approximately TICKS timer ticks.  Interrupts must
   be turned on. */
void timer_sleep(int64_t ticks) {
  int64_t start = timer_ticks();
  ASSERT(intr_get_level() == INTR_ON);

  if (ticks <= 0) return;
  insert_into_sleep_list(thread_current(), start + ticks);
}

/* Sleeps for approximately MS milliseconds.  Interrupts must be
   turned on. */
void timer_msleep(int64_t ms) {
  real_time_sleep(ms, 1000);
}

/* Sleeps for approximately US microseconds.
Interrupts must be turned on. */
void timer_usleep(int64_t us) {
  real_time_sleep(us, 1000 * 1000);
}

/* Sleeps for approximately NS nanoseconds.  Interrupts must be
   turned on. */
void timer_nsleep(int64_t ns) {
  real_time_sleep(ns, 1000 * 1000 * 1000);
}

/* Busy-waits for approximately MS milliseconds.  Interrupts need
   not be turned on.

   Busy waiting wastes CPU cycles, and busy waiting with
   interrupts off for the interval between timer ticks or longer
   will cause timer ticks to be lost.  Thus, use timer_msleep()
   instead if interrupts are enabled. */
void timer_mdelay(int64_t ms) {
  real_time_delay(ms, 1000);
}

/* Sleeps for approximately US microseconds.  Interrupts need not
   be turned on.

   Busy waiting wastes CPU cycles, and busy waiting with
   interrupts off for the interval between timer ticks or longer
   will cause timer ticks to be lost.  Thus, use timer_usleep()
   instead if interrupts are enabled. */
void timer_udelay(int64_t us) {
  real_time_delay(us, 1000 * 1000);
}

/* Sleeps execution for approximately NS nanoseconds.  Interrupts
   need not be turned on.

   Busy waiting wastes CPU cycles, and busy waiting with
   interrupts off for the interval between timer ticks or longer
   will cause timer ticks to be lost.  Thus, use timer_nsleep()
   instead if interrupts are enabled. */
void timer_ndelay(int64_t ns) {
  real_time_delay(ns, 1000 * 1000 * 1000);
}

/* Prints timer statistics. */
void timer_print_stats(void) {
  printf("Timer: %"PRId64" ticks\n", timer_ticks());
}

/* Timer interrupt handler. */
static void timer_interrupt(struct intr_frame *args UNUSED) {
  ticks++;
  thread_tick();
  wake_sleeping_threads();
}

/* Returns true if LOOPS iterations waits for more than one timer
   tick, otherwise false. */
static bool too_many_loops(unsigned loops) {
  int64_t start = ticks;
  while (ticks == start)
    barrier();

  start = ticks;
  busy_wait(loops);

  barrier();
  return start != ticks;
}

/* Iterates through a simple loop LOOPS times, for implementing
   brief delays.

   Marked NO_INLINE because code alignment can significantly
   affect timings, so that if this function was inlined
   differently in different places the results would be difficult
   to predict. */
static void NO_INLINE busy_wait(int64_t loops) {
  while (loops-- > 0)
    barrier();
}

/* Sleep for approximately NUM/DENOM seconds. */
static void real_time_sleep(int64_t num, int32_t denom) {
  int64_t ticks = num * TIMER_FREQ / denom;

  ASSERT(intr_get_level() == INTR_ON);
  if (ticks > 0) {
    timer_sleep(ticks);
  } else {
    real_time_delay(num, denom);
  }
}

/* Busy-wait for approximately NUM/DENOM seconds. */
static void real_time_delay(int64_t num, int32_t denom) {
  ASSERT(denom % 1000 == 0);
  busy_wait(loops_per_tick * num / 1000 * TIMER_FREQ / (denom / 1000));
}
