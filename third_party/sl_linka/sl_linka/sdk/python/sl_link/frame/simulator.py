import asyncio
import logging
from .frame import SlFrame, SlFrameParser, SL_PROTOCOL_VERSION
from .messages import MessageHandler, pb

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class SlLinkSimulator:
    def __init__(self, device_id: int):
        self.device_id = device_id
        self.handler = MessageHandler(device_id)
        self.parser = SlFrameParser()
        self.status_tasks = {}

    @property
    def device_name(self):
        return "LOWER"

    async def _send_periodic_status(self, writer, dst_id=pb.DEVICE_APP):
        logger.info(f"[{self.device_name}] Starting 1Hz status reports to {dst_id}")
        try:
            while True:
                payload, msg_id = self.handler.build_device_status_report()
                frame = SlFrame(
                    version=SL_PROTOCOL_VERSION,
                    flags=0,
                    seq=self.handler.get_next_seq(),
                    ack_seq=0,
                    src_id=self.device_id,
                    dst_id=dst_id,
                    comp_id=pb.COMP_SYSTEM,
                    msg_id=msg_id,
                    payload=payload,
                )
                writer.write(frame.pack())
                await writer.drain()
                await asyncio.sleep(1.0)
        except asyncio.CancelledError:
            logger.info(f"[{self.device_name}] Status task cancelled")

    async def handle_connection(self, reader, writer):
        addr = writer.get_extra_info('peername')
        logger.info(f"[{self.device_name}] New connection from {addr}")
        task = asyncio.create_task(self._send_periodic_status(writer))
        self.status_tasks[writer] = task

        try:
            while True:
                data = await reader.read(2048)
                if not data:
                    break

                frames = self.parser.parse(data)
                for frame in frames:
                    if frame.dst_id not in (self.device_id, pb.DEVICE_BROADCAST):
                        continue

                    logger.info(
                        f"[{self.device_name}] Received msg=0x{frame.msg_id:04X} seq={frame.seq} comp=0x{frame.comp_id:02X}"
                    )

                    resp_payload, resp_msg_id = self.handler.handle(frame)
                    if resp_payload is None:
                        continue

                    resp_frame = SlFrame(
                        version=SL_PROTOCOL_VERSION,
                        flags=0,
                        seq=self.handler.get_next_seq(),
                        ack_seq=frame.seq,
                        src_id=self.device_id,
                        dst_id=frame.src_id,
                        comp_id=frame.comp_id,
                        msg_id=resp_msg_id,
                        payload=resp_payload,
                    )
                    writer.write(resp_frame.pack())
                    await writer.drain()
        finally:
            logger.info(f"[{self.device_name}] Closing connection from {addr}")
            if writer in self.status_tasks:
                self.status_tasks[writer].cancel()
                try:
                    await self.status_tasks[writer]
                except asyncio.CancelledError:
                    pass
                del self.status_tasks[writer]
            writer.close()
            await writer.wait_closed()

    async def start_server(self, host='0.0.0.0', port=8002):
        server = await asyncio.start_server(self.handle_connection, host, port)
        addr = server.sockets[0].getsockname()
        logger.info(f"[{self.device_name}] Serving on {addr}")
        async with server:
            await server.serve_forever()
