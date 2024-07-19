import Logger from '@src/utils/Logger';
import { MAX_HTTP_BUFFER_SIZE } from '@src/utils/constants';
import * as http from 'http';
import * as io from 'socket.io';
import { Service } from 'typedi';
import { FileService } from './file.service';
import { ExplorationService } from './exploration.service';

@Service()
export class SocketManagerService {
    private readonly LOGGER: Logger;
    sio: io.Server;
    users: Map<string, string> = new Map();
    socket: io.Socket;

    constructor(private readonly fileService: FileService, private readonly explorationService: ExplorationService) {
        this.LOGGER = new Logger('SocketManagerService', this.fileService);
    }

    handleSockets(server: http.Server): void {
        this.sio = new io.Server(server, { cors: { origin: '*', methods: ['GET', 'POST'] }, maxHttpBufferSize: MAX_HTTP_BUFFER_SIZE });

        this.sio.on('connection', (socket: io.Socket) => {
            this.socket = socket;
            socket.on('register_user', (username) => {
                console.log(username)
                if (this.users.size < 2 && !Array.from(this.users.values()).includes(username)) {
                    this.users.set(socket.id, username);
                    this.LOGGER.info(`User ${username} connected with socket ID: ${socket.id}`);
                    socket.emit('user_registered', { username: username, id: socket.id });
                    socket.broadcast.emit('user_connected', `User ${username} has connected.`);
                    socket.join('robot_info');

                    if (this.users.size === 1) {
                        socket.emit('set_admin', true);
                    } else {
                        socket.emit('set_admin', false);
                    }

                    this.initializeEventListeners();

                    socket.on('disconnect', () => {
                        this.users.delete(socket.id);
                        this.LOGGER.info(`User ${username} disconnected`);
                        socket.broadcast.emit('user_disconnected', `User ${username} has disconnected.`);
                    });
                } else {
                    socket.emit('registration_failed', { error: "Maximum number of users reached or username already taken" });
                    socket.disconnect(true);
                }
            });
        });
    }

    close(): void {
        this.sio.close();
    }

    async sendLastMission(): Promise<void> {
        const lastMission = await this.explorationService.getLastMission();
        this.socket.emit('last_mission', lastMission);
    }

    private initializeEventListeners(): void {
        this.socket.on('request_history', () => {
            this.sendAllMissions();
        });
        this.socket.on('request_mission_data', async (missionNumber: number) => {
            const missionData = await this.explorationService.getExplorationDetails(missionNumber);
            this.socket.emit('mission_data', missionData);
        });
        this.socket.on('delete_mission', async (missionId: number) => {
            await this.fileService.deleteExploration(missionId);
            this.sendAllMissions();
        });
    }

    async sendAllMissions(): Promise<void> {
        const missions = await this.explorationService.getAllExplorations();
        this.socket.emit('mission_history', missions);
    }

}
