from pycram.language import macros, seq, par

exceptions = []
with seq(exceptions) as state:
    print("typing")
    a = 1
