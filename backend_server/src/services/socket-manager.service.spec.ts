import { expect } from 'chai';
import { SinonStubbedInstance, createStubInstance, stub, restore, spy } from 'sinon';
import { SocketManagerService } from '@src/services/socket-manager.service';
import { FileService } from '@src/services/file.service';
import { ExplorationService } from '@src/services/exploration.service';
import * as http from 'http';
import * as io from 'socket.io';
import { Done } from 'mocha';
import { io as Client, Socket as CSocket } from 'socket.io-client';
import { Subject } from 'rxjs';
import Logger from '@src/utils/Logger';

describe('SocketManagerService', () => {
    let socketManagerService: SocketManagerService;
    let fileServiceStub: SinonStubbedInstance<FileService>;
    let explorationServiceStub: SinonStubbedInstance<ExplorationService>;
    let server: http.Server;
    let clientSocket: CSocket;
    let logger: SinonStubbedInstance<Logger>;

    beforeEach(() => {
        server = http.createServer();
        fileServiceStub = createStubInstance(FileService);
        explorationServiceStub = createStubInstance(ExplorationService);
        logger = createStubInstance(Logger);
        socketManagerService = new SocketManagerService(fileServiceStub, explorationServiceStub);
        Object.defineProperty(socketManagerService, 'LOGGER', {
            value: logger,
            writable: true
        });
    });

    afterEach(() => {
        restore();
    });

    it('should initialize socket.io server', () => {
        socketManagerService.handleSockets(server);
        expect(socketManagerService.sio).to.exist;
    });

    it('should send last mission data to socket', async () => {
        const mockLastMission = {};
        explorationServiceStub.getLastMission.resolves(mockLastMission);

        const socketEmitStub: sinon.SinonStub = stub();
        const socket: Partial<io.Socket> = {
            emit: socketEmitStub
        };
        socketManagerService.socket = socket as io.Socket;

        await socketManagerService.sendLastMission();

        expect(socketEmitStub.calledOnceWith('last_mission', mockLastMission)).to.be.true;
    });

    it('should send all mission data to socket', async () => {
        const mockAllMission = [];
        explorationServiceStub.getAllExplorations.resolves(mockAllMission);

        const socketEmitStub: sinon.SinonStub = stub();
        const socket: Partial<io.Socket> = {
            emit: socketEmitStub
        };
        socketManagerService.socket = socket as io.Socket;

        await socketManagerService.sendAllMissions();

        expect(socketEmitStub.calledOnceWith('mission_history', mockAllMission)).to.be.true;
    });

    it('should handle sockets and disconnect client after registration', (done: Done) => {
        server.listen(() => {
            socketManagerService.handleSockets(server);
            // @ts-ignore
            const port = server.address().port;
            clientSocket = Client(`http://localhost:${port}`);
            
            clientSocket.on('connect', () => {
                clientSocket.emit('register_user', 'user1');
            });
    
            clientSocket.on('disconnect', () => {
                done();
            });
    
            socketManagerService.sio.on('connection', (socket: io.Socket) => {
                socket.on('register_user', (username: string) => {
                    socket.disconnect(true);
                });
            });
        });
    });

    it('should reject registration when maximum number of users is reached', (done: Done) => {
        server.listen(() => {
            socketManagerService.handleSockets(server);
            // @ts-ignore
            const port = server.address().port;
            const clientSocket1 = Client(`http://localhost:${port}`);
            const clientSocket2 = Client(`http://localhost:${port}`);
            const clientSocket3 = Client(`http://localhost:${port}`);
    
            let registeredUsersCount = 0;
    
            clientSocket1.on('connect', () => {
                clientSocket1.emit('register_user', 'user1');
            });
    
            clientSocket2.on('connect', () => {
                clientSocket2.emit('register_user', 'user2');
            });
    
            clientSocket3.on('connect', () => {
                clientSocket3.emit('register_user', 'user3');
            });
    
            clientSocket1.on('disconnect', () => {
                registeredUsersCount++;
                if (registeredUsersCount === 2) {
                    done();
                }
            });
    
            clientSocket2.on('disconnect', () => {
                registeredUsersCount++;
                if (registeredUsersCount === 2) {
                    done();
                }
            });
    
            clientSocket3.on('registration_failed', () => {
                done();
            });
        });
    });
    
    it('should close the socket.io server', (done: Done) => {
        socketManagerService.handleSockets(server);
        
        server.listen(() => {
            // @ts-ignore
            const port = server.address().port;
            const clientSocket = Client(`http://localhost:${port}`);
            
            clientSocket.on('connect', () => {
                expect(socketManagerService.sio).to.exist;

                socketManagerService.close();
                clientSocket.on('disconnect', () => {
                    done();
                });
            });
        });
    });
});
