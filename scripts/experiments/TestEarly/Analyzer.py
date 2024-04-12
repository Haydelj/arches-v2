import matplotlib.pyplot as plt
import numpy as np
import re


# should be modified
scene_list = ["sponza", "san-miguel", "hairball"]
size_list = [256, 1024]
early_list = [0, 1]
delay_list = [0, 1]
scheme_list = [0, 1]

def read_duplication(log_name: str):
    fp = open(log_name)
    ss = fp.readlines()
    ss = [i for i in ss if "Average treelets per ray" in i]
    ans = 0.0
    if len(ss) > 0:
        pattern = re.compile(r'\d+\.\d+')
        ans = pattern.findall(ss[0])
        ans = float(ans[0])
        pass
    ans = round(ans, 2)
    return ans

# for size in size_list:
#     bar_width = 0.25
#     x1 = np.arange(len(scheme_list)) # First bar
#     x2 = x1 + bar_width
#     plt.figure(figsize=(12, 6))
#     for bar_num, scheme in enumerate(scheme_list):
#         y = []
#         for scene in scene_list:

#             # should get correct test_name here, not working now
#             test_name = ""
#             if scheme == "No Early":
#                 test_name = "_" + scene + "_" + "Breadth" + str(size)
#             else:
#                 test_name = "Early" + "_" + scene + "_" + ("Breadth" if scheme == "Breadth First" else "Depth") + str(size)
#             # test_name = ("Early" if scheme != "No Early" else "") + "_" + scene + "_" + ("Breadth" if config["traversal_scheme"] == 0 else "Depth") + str(config["framebuffer_width"])
#             log_name = '{}_log.txt'.format(test_name)
#             y.append(read_duplication(log_name))
#         x = x1 + bar_num * bar_width
#         plt.bar(x, y, width = bar_width, label = scheme)
#         for i in range(len(x)):
#             plt.text(x[i], y[i], y[i], ha='center', va='bottom')

#     plt.title("Framebuffer Size: {}".format(str(size)))
#     plt.xlabel("Scenes", labelpad=10, size=15)
#     plt.ylabel("Ray duplication", size=15)
#     plt.legend()
#     plt.xticks(x1 + bar_width, scene_list, fontsize=15)
#     plt.savefig("{}_result.pdf".format(str(size)), dpi=300)
