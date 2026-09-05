/**
 * Material appearance of the collision meshes.
 *
 * One coherent neutral MATDOG material: medium grey for the chassis and the
 * articulated links, with the four feet a shade darker so the ground contacts
 * read clearly. No per-leg or per-link colour coding — the earlier bright
 * palette existed only to verify that each collision mesh landed on the right
 * link, and that has been verified.
 *
 * Direction cues come from the world axes and arrows instead (X red forward,
 * Y green left, Z blue up), which stay conventionally coloured.
 */

import type { LinkStyle } from '../urdf/buildRobot';

/** Chassis and articulated links. */
const BODY_STYLE: LinkStyle = { color: 0xb2b8bf, metalness: 0.22, roughness: 0.58 };

/** Feet: same material family, slightly darker for contact readability. */
const FOOT_STYLE: LinkStyle = { color: 0x7c838b, metalness: 0.18, roughness: 0.7 };

export function styleForLink(linkName: string): LinkStyle {
  return linkName.endsWith('_foot_link') ? FOOT_STYLE : BODY_STYLE;
}
