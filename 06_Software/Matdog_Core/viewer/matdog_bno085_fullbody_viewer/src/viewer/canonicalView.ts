/** Camera-only framing. +X is the robot's front; neither the model nor IMU is recentered. */
export function canonicalView(aspect: number, headingRad = 0): {
  position: [number, number, number]; target: [number, number, number];
} {
  const safeAspect = Number.isFinite(aspect) && aspect > 0 ? aspect : 1;
  const heading = Number.isFinite(headingRad) ? headingRad : 0;
  // A 45-degree camera: retain enough horizontal room on narrow phone screens
  // without keeping the distant desktop overview on a wide cockpit viewport.
  const distance = Math.max(.68, .25 / (Math.tan(Math.PI / 8) * safeAspect));
  const horizontal = distance * Math.cos(Math.PI / 7);
  return {
    position: [horizontal * Math.cos(heading), horizontal * Math.sin(heading), .08 + distance * Math.sin(Math.PI / 7)],
    target: [0, 0, .08],
  };
}
