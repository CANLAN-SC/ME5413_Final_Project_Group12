### 使用说明：
### 过桥前代码需要一直持续执行，因为需要捕捉话题
1. **触发与订阅**
   - 通过订阅 `/ocr_trigger` (Bool) 话题控制OCR采集，只有收到True信号后，下一帧图像才进行OCR识别。
   - 通过订阅 `/cmd_open_bridge` (Bool) 话题，在收到桥梁解锁信号（True消息）后，停止采集OCR结果，并统计已收集的数字。
2. **OCR识别**
   - 识别有效后，将数字保存到列表中，并发布到 `/recognized_digit` 话题供调试观察。
3. **统计出现次数最少的数字**
   - 在桥梁解锁回调函数中，通过 `collections.Counter` 对所有识别结果进行统计，
   - 利用 `min(counter, key=counter.get)` 计算出出现次数最少的数字，并发布到 `/mode_digit` 话题。