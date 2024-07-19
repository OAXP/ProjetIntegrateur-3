import { TestBed } from '@angular/core/testing';
import { Socket } from 'socket.io-client';
import { SocketClientService } from './socket-client.service';
import { InfoService } from '../info/info.service';
import { RobotInfo } from 'src/app/interfaces/RobotInfo';
import { Buffer } from 'buffer';
import { SocketTestHelper } from './SocketTestHelper';

// Some tests are from the course's gitlab
// Original author is Nikolay Radoev.
describe('SocketClientService', () => {
    let service: SocketClientService;
    let defaultSocket: Socket;
    let socketHelper: SocketTestHelper;
    let mockInfoService: jasmine.SpyObj<InfoService>;

    beforeEach(() => {
        TestBed.configureTestingModule({
            providers: [
              SocketClientService,
              { provide: InfoService, useValue: jasmine.createSpyObj('InfoService', ['updateRobotInfo']) }
            ]
          });
        service = TestBed.inject(SocketClientService);
        mockInfoService = TestBed.inject(InfoService) as jasmine.SpyObj<InfoService>;
        // eslint-disable-next-line no-undef
        socketHelper = new SocketTestHelper();
        service.socket = socketHelper as unknown as Socket;
        defaultSocket = service.socket;
    });

    it('should be created', () => {
        expect(service).toBeTruthy();
    });

    it('should connect if not connected', (done) => {
        spyOn(service, 'isSocketAlive').and.returnValue(false);
        service.connectWithUsername('test').then(() => {
            expect(service.socket).not.toEqual(defaultSocket);
            done();
        });
    });

    it('should not connect if already connected', (done) => {
        spyOn(service, 'isSocketAlive').and.returnValue(true);
        service.connectWithUsername('test').then(() => {
            expect(service.socket).toEqual(defaultSocket);
            done();
        });
    });

    it('should disconnect', () => {
        const spy = spyOn(service.socket, 'disconnect');
        service.disconnect();
        expect(spy).toHaveBeenCalled();
    });

    it('isSocketAlive should return true if the socket is still connected', () => {
        service.socket.connected = true;
        const isAlive = service.isSocketAlive();
        expect(isAlive).toBeTruthy();
    });

    it('isSocketAlive should return false if the socket is no longer connected', () => {
        service.socket.connected = false;
        const isAlive = service.isSocketAlive();
        expect(isAlive).toBeFalsy();
    });

    it('isSocketAlive should return false if the socket is not defined', () => {
        (service.socket as unknown) = undefined;
        const isAlive = service.isSocketAlive();
        expect(isAlive).toBeFalsy();
    });

    describe('Socket events', () => {
        beforeEach(() => {
            service.isTestingEnvironment = true;
            spyOn(service, 'isSocketAlive').and.returnValue(false);
        });

        afterEach(() => {
            service.isTestingEnvironment = false;
        });
    
        it('should emit register_user event on connect', (done) => {
            let spy = spyOn(service.socket, 'emit');
            service.connectWithUsername('test').then(() => {
                expect(spy).toHaveBeenCalledWith('register_user', 'test');
                done();
            });
            socketHelper.simulateEvent('connect');
        });

        it('should handle user_registered event', (done) => {
            let spy = spyOn(service.OnLoginSuccess, 'next');
            service.connectWithUsername('test').then(() => {
                expect(spy).toHaveBeenCalled();
                done();
            });
            socketHelper.simulateEvent('user_registered', { username: 'test' });
        });

        it('should handle registration_failed event', (done) => {
            let spy = spyOn(service.OnLoginError, 'next');
            service.connectWithUsername('test').then(() => {
                expect(spy).toHaveBeenCalled();
                done();
            });
            socketHelper.simulateEvent('registration_failed', { error: 'error' });
        });

        it('should handle disconnect event', (done) => {
            let spy = spyOn(service['userService'], 'clearUser');
            service.connectWithUsername('test').then(() => {
                expect(spy).toHaveBeenCalled();
                done();
            });
            socketHelper.simulateEvent('disconnect');
        });

        it('should handle connect_error event', (done) => {
            let spy = spyOn(service.OnConnectionChanged, 'next');
            service.connectWithUsername('test').then(() => {
                expect(spy).toHaveBeenCalledWith(false);
                done();
            });
            socketHelper.simulateEvent('connect_error', 'error');
        });

        it('should handle user_connected event', (done) => {
            let spy = spyOn(service['xtermLogger'], 'logToXterm');
            service.connectWithUsername('test').then(() => {
                expect(spy).toHaveBeenCalledWith('User connected', 'info');
                done();
            });
            socketHelper.simulateEvent('user_connected', 'User connected');
        });

        it('should handle user_disconnected event', (done) => {
            let spy = spyOn(service['xtermLogger'], 'logToXterm');
            service.connectWithUsername('test').then(() => {
                expect(spy).toHaveBeenCalledWith('User disconnected', 'info');
                done();
            });
            socketHelper.simulateEvent('user_disconnected', 'User disconnected');
        });

        it('should handle set_admin event', (done) => {
            let spy = spyOn(service['userService'], 'setUser');
            const testUser = { username: service['username'], isAdmin: true };
            service.connectWithUsername('test').then(() => {
                expect(spy).toHaveBeenCalledWith(testUser);
                done();
            });
            socketHelper.simulateEvent('set_admin', true);
        });
    });

    describe('Handling Socket Events', () => {
        beforeEach(() => {
          spyOn(console, 'info');
          spyOn(console, 'error');
          spyOn(console, 'log');
        });
      
        it('onRobotInfo should call updateRobotInfo with robotInfo', () => {
          const robotInfo = { robot_id: 'test', battery_volatge: 100, start_time: 0, status: 'test' } as unknown as RobotInfo;
          service.onRobotInfo(robotInfo);
          expect(mockInfoService.updateRobotInfo).toHaveBeenCalledWith(robotInfo.robot_id, robotInfo);
        });
      
        it('onMapInfo should set mapInfo correctly', () => {
          const imgBuffer = new Buffer('test');
          service.onMapInfo(imgBuffer);
          expect(mockInfoService.mapInfo).toEqual('data:image/jpeg;base64,dGVzdA==');
        });
      
        it('onLogInfo should log info level correctly', () => {
          service.onLogInfo({ level: 'info', message: 'Test Info' });
          expect(console.info).toHaveBeenCalledWith('Test Info');
        });
      
        it('onLogInfo should log error level correctly', () => {
          service.onLogInfo({ level: 'error', message: 'Test Error' });
          expect(console.error).toHaveBeenCalledWith('Test Error');
        });
      
        it('onLogInfo should log default level correctly', () => {
          service.onLogInfo({ message: 'Test Default' });
          expect(console.log).toHaveBeenCalledWith('Test Default');
        });
      });
});
