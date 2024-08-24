from gtts import gTTS

# 输入你的英语句子
text = "vision system trained failed"

# 生成语音
tts = gTTS(text, lang='en', tld='com', slow=False)

# 保存为音频文件
tts.save("vision_failed.mp3")

