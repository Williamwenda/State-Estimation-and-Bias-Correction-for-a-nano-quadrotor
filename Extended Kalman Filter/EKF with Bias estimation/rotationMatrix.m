function C = rotationMatrix(quat0, quat1, quat2, quat3)
quat = quaternion(quat0, quat1, quat2, quat3);
C = rotmat(quat,'frame');
end