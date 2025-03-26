import multiprocessing
import time

def worker(process_id):
    print(f"进程{process_id}开始工作")
    time.sleep(2) # 模拟执行任务
    print(f"进程{process_id}工作结束")


processes = []
for i in range(5):
    p = multiprocessing.Process(target=worker,args=(i,))
    processes.append(p)
    p.start() #启动进程


for p in processes:
    p.join() #等待所有进程完成

print("所有进程执行完毕!")
