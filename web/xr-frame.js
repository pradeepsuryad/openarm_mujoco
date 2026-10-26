// Copyright 2026 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Read one WebXR frame into the frame object dora-openarm-webxr's client
// sends to its node (static/ar.js: sendFrame), so that xr-pose.js can be a
// port of the node: { pose_reference, pose_left, pose_right, trigger_*,
// grip_*, joystick_*, button_* }, poses as {x, y, z, qx, qy, qz, qw}.
//
// A tracked bare hand (WebXR Hand Input, which main.js requests as the
// "hand-tracking" feature) fills the same pose_* and trigger_* fields, plus
// hand_* = true, so the rest of the pipeline drives the arm from it as from
// a controller held in that hand: see readHand.

import { quatMul } from "./ik.js";

// Bare hands. WebXR Hand Input gives each joint a pose whose -Z runs along
// the bone toward the fingertips and whose +Y points out of the back of the
// hand. A hand stands in for a controller held in it:
//
// * position: the middle finger's knuckle (middle-finger-phalanx-proximal),
//   which a pinch barely moves, so closing the gripper does not drag the
//   target with it;
// * orientation: the wrist's, turned about its own z so that a hand held
//   thumb up and pointing ahead gives the aim pose a controller held that way
//   does (-Z ahead, +Y up), and CONTROLLER_TO_EE (xr-pose.js) applies as is;
// * trigger: how far the thumb and index fingertips are pinched together.
export const HAND_TO_AIM = {
  left: [Math.SQRT1_2, 0, 0, -Math.SQRT1_2], // -90 degrees about z
  right: [Math.SQRT1_2, 0, 0, Math.SQRT1_2], // +90 degrees about z
};

// Thumb-tip to index-tip distance between the joint centres, in meters, of a
// closed pinch (the fingertips touching) and of an open one. Estimates: hand
// size varies.
export const PINCH_CLOSED = 0.02;
export const PINCH_OPEN = 0.08;

// Pinch distance to a trigger value: 0 open, 1 closed, linear in between.
export function pinchToTrigger(distance) {
  const t = (PINCH_OPEN - distance) / (PINCH_OPEN - PINCH_CLOSED);
  return Math.min(1, Math.max(0, t));
}

// One tracked hand (an XRHand) to the { pose, trigger } a controller held in
// it would give, or null when the headset has lost any of the joints this
// needs: then the side is left out of the frame and its arm holds.
export function readHand(hand, frame, space, handedness) {
  const jointPose = (name) => {
    const joint = hand.get(name);
    return joint ? frame.getJointPose(joint, space) : null;
  };
  const wrist = jointPose("wrist");
  const knuckle = jointPose("middle-finger-phalanx-proximal");
  const thumb = jointPose("thumb-tip");
  const index = jointPose("index-finger-tip");
  if (!wrist || !knuckle || !thumb || !index) return null;
  const o = wrist.transform.orientation;
  const [qw, qx, qy, qz] = quatMul(
    [o.w, o.x, o.y, o.z],
    HAND_TO_AIM[handedness],
  );
  const p = knuckle.transform.position;
  const a = thumb.transform.position;
  const b = index.transform.position;
  return {
    pose: { x: p.x, y: p.y, z: p.z, qx, qy, qz, qw },
    trigger: pinchToTrigger(Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z)),
  };
}

// The controller profiles whose xr-standard gamepad layout is known:
// buttons[0] trigger, [1] squeeze, [4]/[5] A/B on the right hand and X/Y on
// the left.
const KNOWN_PROFILES = ["pico-4u", "meta-quest-touch-plus", "oculus-touch-v3"];

// Unlike ar.js, any other controller with the xr-standard mapping still
// gives its trigger and squeeze, which that mapping fixes at buttons[0] and
// [1], so that its hand moves. Its A/B/X/Y are left out: from buttons[4] on
// the layout is the vendor's, and a guess could reset the environment or
// end the session on the wrong press.
function readTriggers(response, suffix, gamepad) {
  response[`trigger${suffix}`] = gamepad.buttons[0].value;
  // Sent as its 0..1 value rather than a pressed flag, so that a consumer
  // can pick its own threshold.
  const grip = gamepad.buttons[1];
  if (grip) response[`grip${suffix}`] = grip.value;
}

function xrTransform(transform) {
  return {
    x: transform.position.x,
    y: transform.position.y,
    z: transform.position.z,
    qx: transform.orientation.x,
    qy: transform.orientation.y,
    qz: transform.orientation.z,
    qw: transform.orientation.w,
  };
}

export function readFrame(session, space, frame) {
  const response = {};
  // Read before the controller check below: the hand poses are made relative
  // to this pose, so when it is missing the hands are dropped.
  const viewerPose = frame.getViewerPose(space);
  if (viewerPose) response.pose_reference = xrTransform(viewerPose.transform);
  if (session.inputSources.length < 2) return response;
  for (const source of session.inputSources) {
    if (source.handedness === "none") continue;
    const suffix = `_${source.handedness}`;
    if (source.hand && frame.getJointPose) {
      const hand = readHand(source.hand, frame, space, source.handedness);
      if (hand) {
        response[`pose${suffix}`] = hand.pose;
        response[`trigger${suffix}`] = hand.trigger;
        response[`hand${suffix}`] = true;
      }
      continue;
    }
    // The target ray space is the OpenXR aim pose: its -Z points where the
    // controller points, which xr-pose.js maps onto the gripper axis. Not
    // the grip pose, whose -Z runs along the handle and would turn the
    // handle's tilt into the gripper's.
    const pose = frame.getPose(source.targetRaySpace, space);
    if (pose) response[`pose${suffix}`] = xrTransform(pose.transform);
    const gamepad = source.gamepad;
    if (!gamepad) continue;
    if (source.profiles.some((p) => KNOWN_PROFILES.includes(p))) {
      readTriggers(response, suffix, gamepad);
      if (source.handedness === "right") {
        response.button_a = gamepad.buttons[4].pressed;
        response.button_b = gamepad.buttons[5].pressed;
      } else {
        response.button_x = gamepad.buttons[4].pressed;
        response.button_y = gamepad.buttons[5].pressed;
      }
    } else if (gamepad.mapping === "xr-standard" && gamepad.buttons[0]) {
      readTriggers(response, suffix, gamepad);
    }
    // The whole array: the xr-standard mapping reserves the first axis
    // pair for the touchpad and the second for the thumbstick, and a
    // centred stick reads exactly 0, so no truthiness check here.
    if (gamepad.axes.length >= 2) {
      response[`joystick${suffix}`] = Array.from(gamepad.axes);
    }
  }
  return response;
}
