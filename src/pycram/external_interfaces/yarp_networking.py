
import yarp


ACK_VOCAB = yarp.createVocab32('a','c','k')
NO_ACK_VOCAB = yarp.createVocab32('n','a','c','k')


def init_yarp_network():
    if not yarp.Network.checkNetwork():
        print("Unable to find a yarp server exiting ...")
        return False

    yarp.Network.init()
    return True

def open_rpc_client_port(port_name):
    handle_port: yarp.RpcClient = yarp.RpcClient()
    if not handle_port.open(port_name):
        print(f"Can't open the port %s correctly" % port_name)
        return False , None
    print(f"Port %s opened correctly" % port_name)
    return True , handle_port

def open_buffered_bottle_port(port_name):
    opened_port: yarp.BufferedPortBottle = yarp.BufferedPortBottle()
    if not opened_port.open(port_name):
        print(f"Can't open the port %s correctly" % port_name)
        return False , None
    print(f"Port %s opened correctly" % port_name)
    return True , opened_port


def interrupt_and_close(m_module:yarp.RFModule):
    m_module.interruptModule()
    m_module.close()
