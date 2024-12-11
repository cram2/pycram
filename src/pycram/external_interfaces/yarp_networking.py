from typing_extensions import Tuple,Optional

from pycram.ros.logging import logwarn

try:
    import yarp
except ImportError:
    logwarn("yarp wasn't found, please check the installation")

if yarp is not None:

    ACK_VOCAB = yarp.createVocab32('a','c','k')
    NO_ACK_VOCAB = yarp.createVocab32('n','a','c','k')


    def init_yarp_network():
        """
         Initializes the YARP network.

         :param func: Function that should be thread safe
         :return bool -> true  if the YARP network is successfully initialized.
         """
        if not yarp.Network.checkNetwork():
            print("Unable to find a yarp server exiting ...")
            return False

        yarp.Network.init()
        return True

    def open_rpc_client_port(port_name:str)->Tuple[bool, Optional[yarp.RpcClient]]:
        """
            Opens a YARP RpcClient port with the specified name.

            :param `port_name` (str): The name of the RPC client port to be opened.
            :return Tuple: (bool ( success/ fail ), yarp.RpcClient or None)
        """
        handle_port: yarp.RpcClient = yarp.RpcClient()
        if not handle_port.open(port_name):
            print(f"Can't open the port %s correctly" % port_name)
            return False , None
        print(f"Port %s opened correctly" % port_name)
        return True , handle_port

    def open_buffered_bottle_port(port_name:str)->Tuple[bool, Optional[yarp.BufferedPortBottle]]:
        """
            Opens a YARP BufferedPortBottle with the specified port name.

            :param `port_name` (str): The name of the port to be opened.
            :return Tuple: (bool ( success/ fail ), yarp.BufferedPortBottle or None)
        """
        opened_port: yarp.BufferedPortBottle = yarp.BufferedPortBottle()
        if not opened_port.open(port_name):
            print(f"Can't open the port %s correctly" % port_name)
            return False , None
        print(f"Port %s opened correctly" % port_name)
        return True , opened_port


    def interrupt_and_close(m_module:yarp.RFModule):
        """
             interrupt and close the module
             :param m_module: yarp module
        """
        m_module.interruptModule()
        m_module.close()
        print("iCub state updater closed")
