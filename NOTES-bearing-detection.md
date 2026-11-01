# BearingDetection: notes for SpiriDetect

`BearingDetection` in `src/spiri_mav/gimbal.py` is a sample, thrown together
from the gimbal rangefinder work. It's the seed of SpiriDetect's first
detection kind and moves there once that repo exists. These notes are what
it has, and what the real one needs.

## What the sample does

- A `Position` under the gimbal (`<robot>_gimbal<id>_range`): a point at the
  reading's distance along the gimbal's x axis, so it has a pose and ECEF
  coordinates like any frame.
- Its mount's `Accuracy` is the beam's error (`range_accuracy` along it,
  `pointing_accuracy` off it, split horizontal/vertical by the beam's
  elevation; `beam_accuracy`), so SpiriSynq sums the gimbal's and robot's
  accuracy down the tree into its `pose.accuracy`.
- Metadata: `status` (`ok`, `near`, `far`, or `None` when silent),
  `distance`, the sensor's `min`/`max`, and the beam accuracies.
- The debug map draws it as the one exception to "every Position alike": a
  cone along the gimbal's beam, rings at the ± range, a marker at the point,
  a distance label that opens to the usual frame details and 90% cylinder.

## What a real BearingDetection needs

- **Live, as it is.** A detection is an ordinary object in the pose tree: one
  object per thing, updated in place as it moves, removed when it's gone.
  The sample already works this way, following the gimbal at 10 Hz. (The
  charter's "immutable, timestamped claim, never edited" model predates
  SpiriSynq and is outdated.)
- **Accuracy as in Cursor on Target.** Our accuracy model is CoT's at 90%
  instead of 1σ (`Accuracy.to_cot()` / `from_cot()` convert), so a detection
  round-trips through TAK; see SpiriTak's `<gcs>/detections/takbridge`.
- **Confidence**, where the source has one; "not given" is a valid value,
  and the sample has none.
- **A bearing without a range.** The general case is a ray from a sensor pose
  with angular uncertainty: RF direction finding, acoustic, a camera pixel.
  With a range it collapses to a point with an `Accuracy`; without, it's a
  cone (the map already draws one).

## The box kind (YOLO)

A box detection can be a BearingDetection too: through the camera's
intrinsics (`Camera.intrinsics`), the box centre is a bearing and its width
and height are its angular footprint. Carry the pixel box, class and camera
link as metadata; no range, so the map draws a small frustum, like camera
coverage. A known-size prior (a person is ~1.7 m) could give a rough range
with honestly large bounds, later and optionally.
