import math

def cacheLastResult(func):
    '''
        Function decorator that caches that function's last return value.
            Could use @functools.lru_cache(maxsize=1) to achieve the same behavior, but probably slower
    '''
    cache = dict()
    cache[1] = [] # Cache last argument list here
    cache[2] = None # Cache last result here

    def memoized_func(*args):
        # Return cached result if available
        if cache[1] == args:
            return cache[2]
        
        # Compute and cache result
        result = func(*args)
        cache[1] = args
        cache[2] = result
        return result

    return memoized_func

def evalExpression(statement: str, additionalVars={}):
    globalVars = {
        'math': math # Make math functions available
    }

    return eval(statement, globalVars, additionalVars)
