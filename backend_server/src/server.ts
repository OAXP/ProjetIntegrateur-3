import { Application } from '@src/app';
import * as http from 'http';
import { AddressInfo } from 'net';
import { Service } from 'typedi';
import 'dotenv/config';
import Logger from '@src/utils/Logger';
import { SocketManagerService } from './services/socket-manager.service';
import { InfoService } from './services/info.service';
import { MapService } from './services/map.service';
import { RosService } from './services/ros.service';
import { FileService } from './services/file.service';
import LogSocketManager from './utils/LogSocketManager';

@Service()
export class Server {
    private static readonly appPort: string | number | boolean =
        Server.normalizePort(process.env.PORT || '3000');
    // eslint-disable-next-line @typescript-eslint/no-magic-numbers
    private static readonly baseDix: number = 10;
    private server: http.Server;
    private readonly LOGGER: Logger;

    constructor(
        private readonly application: Application,
        private readonly socketManagerService: SocketManagerService,
        private readonly infoService: InfoService,
        private readonly mapService: MapService,
        private readonly rosService: RosService,
        private readonly fileService: FileService,
    ) {
        this.LOGGER = new Logger('Server', fileService);
    }

    private static normalizePort(
        val: number | string
    ): number | string | boolean {
        const port: number =
            typeof val === 'string' ? parseInt(val, this.baseDix) : val;
        if (isNaN(port)) {
            return val;
        } else if (port >= 0) {
            return port;
        } else {
            return false;
        }
    }
    init(): void {
        this.application.app.set('port', Server.appPort);

        this.server = http.createServer(this.application.app);

        this.socketManagerService.handleSockets(this.server);

        this.rosService.initialize().then(() => {
            this.infoService.init();
            this.mapService.init();
        });

        this.server.listen(Server.appPort);
        this.server.setTimeout(10000);
        this.server.on('error', (error: NodeJS.ErrnoException) =>
            this.onError(error)
        );
        this.server.on('listening', () => this.onListening());
        new LogSocketManager(this.socketManagerService);
    }

    private onError(error: NodeJS.ErrnoException): void {
        if (error.syscall !== 'listen') {
            throw error;
        }
        const bind: string =
            typeof Server.appPort === 'string'
                ? 'Pipe ' + Server.appPort
                : 'Port ' + Server.appPort;
        switch (error.code) {
            case 'EACCES':
                // eslint-disable-next-line no-console
                console.error(`${bind} requires elevated privileges`);
                process.exit(1);
                break;
            case 'EADDRINUSE':
                // eslint-disable-next-line no-console
                console.error(`${bind} is already in use`);
                process.exit(1);
                break;
            default:
                throw error;
        }
    }

    /**
     * Se produit lorsque le serveur se met à écouter sur le port.
     */
    private onListening(): void {
        const addr = this.server.address() as AddressInfo;
        const bind: string =
            typeof addr === 'string' ? `pipe ${addr}` : `port ${addr.port}`;
        // eslint-disable-next-line no-console
        console.log(`Listening on ${bind}`);
    }
}
