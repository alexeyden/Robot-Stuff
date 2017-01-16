import json
import multiprocessing as mp

from http.server import HTTPServer, BaseHTTPRequestHandler


class Handler(BaseHTTPRequestHandler):
    _pipe = None

    def do_POST(self):
        text = self.rfile.read(int(self.headers['Content-Length'])).decode('utf-8')
        data = json.loads(text)
        print(data)
        self._pipe.send(data)
        resp = self._pipe.recv()

        js_resp = json.dumps(resp)

        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()

        self.wfile.write(js_resp.encode('utf-8'))


class Server:
    def __init__(self, port, pipe):
        self.port = port
        self.pipe = pipe

    @staticmethod
    def _process(pipe):
        Handler._pipe = pipe
        server = HTTPServer(('localhost', 8080), Handler)
        server.serve_forever()

    def run(self):
        self._proc = mp.Process(target=self._process, args=(self.pipe, ))
        self._proc.start()

    def stop(self):
        self._proc.terminate()
        self._proc.join()
