from timeit import timeit

nTests = 100000
print("nTests: {}".format(nTests))

print("\n ----Regex----")
regexSetup = '''
import re
dictStartLine = re.compile("^[\s]*.*{[\s]*$")
s = "   dictionaryName{   "
'''
print(timeit("re.match(dictStartLine, s) != None", setup=regexSetup, number=nTests))

print("\n ----Python----")
pySetup = '''
s = "   dictionaryName{   "
'''
print(timeit("s.strip()[-1] == '{'", setup=pySetup, number=nTests))