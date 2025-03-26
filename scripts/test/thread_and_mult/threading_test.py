import threading
import time


def worker(thread_id):
    print(f"线程{thread_id}开始工作")
    time.sleep(2)
    print(f"线程{thread_id}工作结束")


threads = []

for i in range(5):
    t = threading.Thread(target=worker,args=(i,))
    threads.append(t)
    t.start() #启动线程

for t in threads:
    t.join() #等待所有线程完成

print("所有线程执行完毕!")
