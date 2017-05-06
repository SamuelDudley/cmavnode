import os, time
import libcmavnodepy as cmavnode # cmavnode python wrapper

current_dir = os.path.dirname(os.path.abspath(__file__))
router = cmavnode.Wrapper(os.path.join(os.path.dirname(current_dir),"example","test.conf"))
router.configureLogger(os.path.join(os.path.dirname(current_dir), 'log.conf'))
while True:
    print "shell 'help': "
    router.shell('help')
    
    print "shell 'stat': "
    router.shell('stat')
    
    print "direct call to stats: "
    router.printLinkStats()
    
    time.sleep(1)
    
# now that we have the router object we can control and monitor it from python
# e.g. parse json in python from a mult-processing queue and apply them to cmavnode