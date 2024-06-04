# 'Red' = our ball
# 'Blue' = oppenent's ball
# [0,1,2,3,4]

        #center     #left     #right    #leftleft  #rightright
test = [["Red","Blue","Red"],    ["None","Red","Blue"],    ["None","Red","Blue"]]

        #center     #left     #right    #leftleft  #rightright
test = [[],    [],    [],    [],    []] 

class Decision:
    # shortest_path_list = [4,0,3,1,2]
    # shortest_path_state = 0
    def __init__(self, silo, color):
        self.silo = silo
        self.idx = -1
        self.shortest_path_list = [4,0,3,1,2]
        #self.shortest_path_list = [2,0,1]
        self.shortest_path_state = 0
        self.color = color

    def silo_decision(self):
        # Priority
        pr1 = 5
        pr2 = 5
        pr3 = 5
        count = 0
        if self.color == "red":
            for i in self.silo:
                if "None" not in i:
                    count += 1
            if count == 5:
                return 10000000 
            if "None" in self.silo[self.shortest_path_list[self.idx]]:
                self.shortest_path_state = self.shortest_path_list[self.idx]
                return self.shortest_path_state
            else:
                self.idx -= 1
                self.shortest_path_state = self.shortest_path_list[self.idx]
                return self.shortest_path_state
            
        elif self.color == "blue":
            for i in self.silo:
                if "None" not in i:
                    count += 1
            if count == 5:
                return 10000000 
            if "None" in self.silo[self.shortest_path_list[self.idx]]:
                self.shortest_path_state = self.shortest_path_list[self.idx]
                return self.shortest_path_state
            else:
                self.idx -= 1
                self.shortest_path_state = self.shortest_path_list[self.idx]
                return self.shortest_path_state
            
        else:
            return "Wrong input"



# import json
# import time
# # 'Red' = our ball
# # 'Blue' = oppenent's ball
# # [0,1,2,3,4]

#         #center     #left     #right    #leftleft  #rightright
# test = [['Red','Blue','Red'],    ["None",'Red','Blue'],    ["None",'Red','Blue'],    ["None","None","None"],     ["None",'Blue','Blue']]

#         #center     #left     #right    #leftleft  #rightright
# test = [[],    [],    [],    [],     []]

# class Decision:
#     # shortest_path_list = [4,0,3,1,2]
#     # shortest_path_state = 0
#     def __init__(self, silo):
#         self.silo = silo
#         self.idx = 0
#         self.shortest_path_list = [4,0,3,1,2]
#         self.shortest_path_state = 0

#     def silo_decision(self):
#         # Priority
#         pr1 = 5
#         pr2 = 5
#         pr3 = 5
#         pr4 = 5
#         pr5 = 5
#         pr6 = 5
#         if (['None','Red','Blue'] in self.silo) or (['None','Blue','Red'] in self.silo):
#             print('One I One enermy')
#             if (['None','Red','Blue'] in self.silo):
#                 pr1 = self.silo.index(['None','Red','Blue'])
#             if (['None','Blue','Red'] in self.silo):
#                 pr2 = self.silo.index(['None','Blue','Red'])
#             if pr1 < pr2:
#                 print(f"['Red','Blue'] : {pr1}")
#                 return pr1
#             else:
#                 print(f"['Blue,'Red'] : {pr2}")
#                 return pr2
            
#         elif (['None','Blue','Blue'] in self.silo):
#             print('two enermy')
#             pr3 = self.silo.index(['None','Blue','Blue'])
#             print(f"['Blue','Blue'] : {pr3}")
#             return pr3
        
#         elif (['None','None','Blue'] in self.silo):
#             if (['None','None','None'] in self.silo):
#                 print("pr4")
#                 pr4 = self.silo.index(['None','None','None'])
#                 return pr4
            
#         elif (['None','None','None'] not in self.silo):
#             if (['None','None','Red'] in self.silo):
#                 print("pr5")
#                 pr5 = self.silo.index(['None','None','Red'])
#                 return pr5
#             elif (['None','Red','Red'] in self.silo):
#                 print("pr6")
#                 pr6 = self.silo.index(['None','Red','Red'])
#                 return pr6
#         else:
#             print('the nearest')
#             self.idx -= 1
#             self.shortest_path_state = self.shortest_path_list[self.idx]
#             if 'None' not in self.silo[self.shortest_path_list[self.idx]]:
#                 self.idx -= 1
#             else:
#                 return self.shortest_path_state

# what = Decision(test)
# print(what.silo_decision())
# print("----------------------------")

# # t = 0
# # testcase = [[['None','None','None'],  ['None','None','None'],   ['None','None','Blue'],   ['None','None','Red'],     ['None','None','Blue']],
# #             [['None','None','None'],  ['None','None','None'],   ['None','None','None'],   ['None','Blue','Blue'],     ['None','None','None']],
# #             [['None','None','None'],  ['None','None','None'],   ['None','None','None'],   ['None','None','None'],     ['None','Blue','Blue']],
# #             [['None','None','None'],  ['None','None','None'],   ['None','None','None'],   ['None','None','None'],     ['None','Red','Blue']],
# #             [['None','None','None'],  ['None','None','None'],   ['None','None','None'],   ['None','Blue','Red'],     ['None','None','None']],
# #             [['None','None','None'],  ['None','None','None'],   ['None','Red','Blue'],   ['None','None','None'],     ['None','None','None']]]
# # while t < 6:
# #     print("t : ", t)
# #     time.sleep(1)
# #     obj = Decision(testcase[t])
# #     print("idx = ", obj.idx)
# #     print("select Silo no. : ",obj.silo_decision())
# #     print(" ")
# #     t += 1  
# # a= [1,2,3]
# # a.pop(2)
# # print(f"athit : {a}")