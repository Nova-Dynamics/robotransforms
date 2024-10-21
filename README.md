## Robotransforms
A transformation library for robot motion

## Installation
```bash
npm install @novadynamics/robotransforms
```

## Usage
```javascript
const t = require('robotransforms').euclidean;
// Example inputs
const euler_angles = [0.1, 0.2, 0.3];
const vector = [1, 0, 0];

// Conversions
const rotation_matrix = t.euler2rotmat(euler_angles);
const euler_from_rotmat = t.rotmat2euler(rotation_matrix);
const quaternion = t.euler2quat(euler_angles);
const red_quat = t.quat2redquat(quaternion);
const rot_vec = t.quat2rotvec(quaternion);
const quat_from_rotvec = t.rotvec2quat(rot_vec);
const euler_from_quat = t.quat2euler(quaternion);

// Applications
const rotated_vector_rotmat = t.apply_rotmat(rotation_matrix, vector);
const rotated_vector_quat = t.apply_quat(quaternion, vector);
const rotated_vector_redquat = t.apply_redquat(red_quat, vector);
const rotated_vector_rotvec = t.apply_rotvec(rot_vec, vector);
const rotated_vector_euler = t.apply_euler(euler_angles, vector);

// Inversions
const inv_rotation_matrix = t.invert_rotmat(rotation_matrix);
const inv_quaternion = t.invert_quat(quaternion);
const inv_redquat = t.invert_redquat(red_quat);
const inv_rotvec = t.invert_rotvec(rot_vec);
const inv_euler = t.invert_euler(euler_angles);

// Compositions
const composed_rotmat = t.compose_rotmat(rotation_matrix, rotation_matrix);
const composed_quat = t.compose_quat(quaternion, quaternion);
const composed_redquat = t.compose_redquat(red_quat, red_quat);
const composed_rotvec = t.compose_rotvec(rot_vec, rot_vec);
const composed_euler = t.compose_rotmat(rotation_matrix, rotation_matrix);

// Print results
console.log("Euler angles:", euler_angles);
console.log("Rotation Matrix:\n", rotation_matrix);
console.log("Euler from Rotation Matrix:", euler_from_rotmat);
console.log("Quaternion:", quaternion);
console.log("Reduced Quaternion:", red_quat);
console.log("Rotation Vector:", rot_vec);
console.log("Quaternion from Rotation Vector:", quat_from_rotvec);
console.log("Euler from Quaternion:", euler_from_quat);
console.log("\nRotated Vector (Rotation Matrix):", rotated_vector_rotmat);
console.log("Rotated Vector (Quaternion):", rotated_vector_quat);
console.log("Rotated Vector (Reduced Quaternion):", rotated_vector_redquat);
console.log("Rotated Vector (Rotation Vector):", rotated_vector_rotvec);
console.log("Rotated Vector (Euler):", rotated_vector_euler);
console.log("\nInverse Rotation Matrix:\n", inv_rotation_matrix);
console.log("Inverse Quaternion:", inv_quaternion);
console.log("Inverse Reduced Quaternion:", inv_redquat);
console.log("Inverse Rotation Vector:", inv_rotvec);
console.log("Inverse Euler angles:", inv_euler);
console.log("\nComposed Rotation Matrix:\n", composed_rotmat);
console.log("Composed Quaternion:", composed_quat);
console.log("Composed Reduced Quaternion:", composed_redquat);
console.log("Composed Rotation Vector:", composed_rotvec);
console.log("Composed Euler angles:", composed_euler);

```