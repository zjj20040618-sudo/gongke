
[basic]
type = axmodel
model_npu = model_9767_npu.axmodel
model_vnpu = model_9767_vnpu.axmodel

[extra]
model_type = yolo26
type=detector
input_type = rgb

input_cache = true
output_cache = true
input_cache_flush = false
output_cache_inval = true

labels = 扁圆物体, 圆柱体, 圆台体, 蓝球, 红球, 绿球, 红靶子, 蓝靶子, 绿靶子, 黑桶
mean = 0, 0, 0
scale = 0.00392156862745098, 0.00392156862745098, 0.00392156862745098

