# -*- coding:utf-8 -*-
'''
CPU_and_MEM_Monitor
思路：后端后台线程一旦产生数据，即刻推送至前端。
好处：不需要前端ajax定时查询，节省服务器资源。
'''

import psutil    #这个库可以用来获取系统的资源数据，详细可以看文档
import time

from threading import Lock
import json
from flask import Flask, render_template, session, request
from flask_socketio import SocketIO, emit

# Set this variable to "threading", "eventlet" or "gevent" to test the
# different async modes, or leave it set to None for the application to choose
# the best option based on installed packages.
async_mode = None

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, async_mode=async_mode)


thread = None
thread_lock = Lock()



# 后台线程 产生数据，即刻推送至前端
def background_thread():
    count = 0
    while True:
        socketio.sleep(0.2)
        count += 1
        t = time.strftime('%M:%S', time.localtime()) # 获取系统时间（只取分:秒）
        cpus = psutil.cpu_percent(interval=None, percpu=True) # 获取系统cpu使用率 non-blocking
        socketio.emit('server_response',
                      {'data': [t] + list(cpus)[0:4], 'count': count},
                      namespace='/test') # 注意：这里不需要客户端连接的上下文，默认 broadcast = True ！！！！！！！
        print([t] +list(cpus)[0:4])
        print(100*'*')

# 当用户访问'/'时，执行index()函数。这也是python装饰器的用法。
@app.route('/')
def index():
    return render_template('index.html', async_mode=socketio.async_mode)
    # 每次执行render_template函数时，渲染器都会将index.html的变量值用其实际值替代。


# 与前端建立 socket 连接后，启动后台线程
@socketio.on('connect', namespace='/test')
def test_connect():
    global thread
    with thread_lock:
        if thread is None:
            thread = socketio.start_background_task(target=background_thread)




if __name__ == '__main__':
    socketio.run(app, debug=True)

