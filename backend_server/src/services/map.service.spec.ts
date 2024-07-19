import { expect } from 'chai';
import { SinonStubbedInstance, createStubInstance, stub, restore, SinonStub } from 'sinon';
import { MapService } from '@src/services/map.service';
import { RosService } from '@src/services/ros.service';
import { FileService } from '@src/services/file.service';
import { SocketManagerService } from '@src/services/socket-manager.service';
import * as rclnodejs from 'rclnodejs';
import * as io from 'socket.io';
import { InfoService } from './info.service';

describe('MapService', () => {
    let mapService: MapService;
    let rosServiceStub: SinonStubbedInstance<RosService>;
    let fileServiceStub: SinonStubbedInstance<FileService>;
    let socketManagerServiceStub: SinonStubbedInstance<SocketManagerService>;
    let infoServiceStub: SinonStubbedInstance<InfoService>;

    beforeEach(() => {
        rosServiceStub = createStubInstance(RosService);
        infoServiceStub = createStubInstance(InfoService);
        rosServiceStub.getNode.returns({
            createSubscription: stub((type: string, topic: string, callback: SinonStub) => {
                callback();
            }),
        } as unknown as rclnodejs.Node);
        fileServiceStub = createStubInstance(FileService);
        socketManagerServiceStub = createStubInstance(SocketManagerService);
        socketManagerServiceStub.sio = {
            emit: stub().returnsThis(),
        } as unknown as io.Server;
        mapService = new MapService(rosServiceStub, socketManagerServiceStub, fileServiceStub, infoServiceStub);
    });

    afterEach(() => {
        restore();
    });

    it('should create subscriptions to map topics', () => {
        mapService['map1_callback'] = stub();
        mapService['map2_callback'] = stub();

        mapService.init();

        expect(rosServiceStub.getNode.called).to.be.true;
    });
    

    it('should set map1 and transmit mergedMap if map2 is available', () => {
        const transmitMapStub = stub(mapService, 'transmitMap' as keyof MapService);
        const mergeMapStub = stub(mapService, 'mergeMap' as keyof MapService);

        const map1 = {} as rclnodejs.nav_msgs.msg.OccupancyGrid;
        const map2 = {} as rclnodejs.nav_msgs.msg.OccupancyGrid;

        mapService['map2'] = map2;

        mapService['map1_callback'](map1);

        expect(mapService['map1']).to.equal(map1);
        expect(transmitMapStub.calledOnce).to.be.true;
        expect(mergeMapStub.calledOnce).to.be.true;
    });

    it('should set and transmit map1 if map2 is unavailable', () => {
        const transmitMapStub = stub(mapService, 'transmitMap' as keyof MapService);
        const mergeMapStub = stub(mapService, 'mergeMap' as keyof MapService);

        const map1 = {} as rclnodejs.nav_msgs.msg.OccupancyGrid;

        mapService['map2'] = null;

        mapService['map1_callback'](map1);

        expect(mapService['map1']).to.equal(map1);
        expect(transmitMapStub.calledOnce).to.be.true;
        expect(mergeMapStub.calledOnce).to.be.false;
    });

    it('should set map2 and transmit mergedMap if map1 is available', () => {
        const transmitMapStub = stub(mapService, 'transmitMap' as keyof MapService);
        const mergeMapStub = stub(mapService, 'mergeMap' as keyof MapService);

        const map1 = {} as rclnodejs.nav_msgs.msg.OccupancyGrid;
        const map2 = {} as rclnodejs.nav_msgs.msg.OccupancyGrid;

        mapService['map1'] = map1;

        mapService['map2_callback'](map2);

        expect(mapService['map2']).to.equal(map2);
        expect(transmitMapStub.calledOnce).to.be.true;
        expect(mergeMapStub.calledOnce).to.be.true;
    });

    it('should set and transmit map2 if map1 is unavailable', () => {
        const transmitMapStub = stub(mapService, 'transmitMap' as keyof MapService);
        const mergeMapStub = stub(mapService, 'mergeMap' as keyof MapService);
        
        const map2 = {} as rclnodejs.nav_msgs.msg.OccupancyGrid;

        mapService['map1'] = null;

        mapService['map2_callback'](map2);

        expect(mapService['map2']).to.equal(map2);
        expect(transmitMapStub.calledOnce).to.be.true;
        expect(mergeMapStub.calledOnce).to.be.false;
    });

    it('should merge map', () => {
        const map1: rclnodejs.nav_msgs.msg.OccupancyGrid = {
            header: {
                stamp: { sec: 0, nanosec: 0 }, 
                frame_id: 'map1_frame',
            },
            info: {
                map_load_time: { sec: 0, nanosec: 0 }, 
                resolution: 0.1,
                width: 10,
                height: 10,
                origin: {
                    position: { x: 0, y: 0, z: 0 },
                    orientation: { x: 0, y: 0, z: 0, w: 1 },
                },
            },
            data: new Array(100).fill(0),
        };
        const map2: rclnodejs.nav_msgs.msg.OccupancyGrid = {
            header: {
                stamp: { sec: 0, nanosec: 0 }, 
                frame_id: 'map2_frame',
            },
            info: {
                map_load_time: { sec: 0, nanosec: 0 }, 
                resolution: 0.1,
                width: 10,
                height: 10,
                origin: {
                    position: { x: 5, y: 5, z: 0 },
                    orientation: { x: 0, y: 0, z: 0, w: 1 },
                },
            },
            data: new Array(100).fill(0),
        };

        const mergedMap = mapService['mergeMap'](map1, map2);
        expect(mergedMap.header.frame_id).to.equal('merge_map');
        expect(mergedMap.header).to.equal(map1.header);
    });

    it('should transmit map', () => {
        const fillRectStub = stub(mapService['ctx'], 'fillRect').callsFake(() => {});
        const map: rclnodejs.nav_msgs.msg.OccupancyGrid = {
            header: {
                stamp: { sec: 0, nanosec: 0 }, 
                frame_id: 'map2_frame',
            },
            info: {
                map_load_time: { sec: 0, nanosec: 0 }, 
                resolution: 0.1,
                width: 10,
                height: 10,
                origin: {
                    position: { x: 5, y: 5, z: 0 },
                    orientation: { x: 0, y: 0, z: 0, w: 1 },
                },
            },
            data: new Array(100).fill(0),
        };

        mapService['transmitMap'](map);

        expect(fillRectStub.called).to.be.true;
    });

    it('should not transmit map when not exploring', () => {
        const fillRectStub = stub(mapService['ctx'], 'fillRect').callsFake(() => {});
        const map: rclnodejs.nav_msgs.msg.OccupancyGrid = {
            header: {
                stamp: { sec: 0, nanosec: 0 }, 
                frame_id: 'map2_frame',
            },
            info: {
                map_load_time: { sec: 0, nanosec: 0 }, 
                resolution: 0.1,
                width: 10,
                height: 10,
                origin: {
                    position: { x: 5, y: 5, z: 0 },
                    orientation: { x: 0, y: 0, z: 0, w: 1 },
                },
            },
            data: new Array(100).fill(0),
        };

        fileServiceStub['logFolderPath'] = '';
        mapService['transmitMap'](map);

        expect(fillRectStub.called).to.be.false;
    });

    it('should log error', () => {
        const toBufferStub = stub(mapService['canvas'], 'toBuffer').callsArgWith(0, new Error('Failed to turn to buffer (test)'));
        const logErrorStub = stub(mapService['LOGGER'], 'err');
    
        const map: rclnodejs.nav_msgs.msg.OccupancyGrid = {
            header: {
                stamp: { sec: 0, nanosec: 0 }, 
                frame_id: 'map2_frame',
            },
            info: {
                map_load_time: { sec: 0, nanosec: 0 }, 
                resolution: 0.1,
                width: 10,
                height: 10,
                origin: {
                    position: { x: 5, y: 5, z: 0 },
                    orientation: { x: 0, y: 0, z: 0, w: 1 },
                },
            },
            data: new Array(100).fill(70),
        };
    
        fileServiceStub['logFolderPath'] = 'exploration-1';
        mapService['transmitMap'](map);
    
        expect(logErrorStub.calledOnce).to.be.true;
    });
    

    it('should clear canvas', () => {
        const clearReactStub = stub(mapService['ctx'], 'clearRect').callsFake(() => {});

    
        mapService.clearCanvas();

        expect(clearReactStub.called).to.be.true;
    });
});

