import { SinonStubbedInstance, createStubInstance, stub, restore } from 'sinon';
import { expect } from 'chai';
import * as rclnodejs from 'rclnodejs';
import { RosService } from '@src/services/ros.service';
import { FileService } from '@src/services/file.service';
import Logger from "@src/utils/Logger";

describe('RosService', () => {
    let rosService: RosService;
    let fileServiceStub: SinonStubbedInstance<FileService>;
    let initStub: sinon.SinonStub;
    let shutdownStub: sinon.SinonStub;
    let nodeStub: SinonStubbedInstance<rclnodejs.Node>;
    let logger: SinonStubbedInstance<Logger>;

    beforeEach(() => {
        fileServiceStub = createStubInstance(FileService);
        initStub = stub(rclnodejs, 'init'); 
        shutdownStub = stub(rclnodejs, 'shutdown'); 
        rosService = new RosService(fileServiceStub);
        nodeStub = createStubInstance(rclnodejs.Node);
        logger = createStubInstance(Logger);
        Object.defineProperty(rosService, 'LOGGER', {
            value: logger,
            writable: true
        });
    });

    afterEach(() => {
        restore();
    });

    it('should initialize ROS context successfully', async () => {
        await rosService.initialize();

        expect(initStub.calledOnce).to.be.true;
        expect(rosService.initialized).to.be.true;
    });

    it('should log error if initialization fails', async () => {
        const errorMessage = 'Initialization failed';
        initStub.rejects(new Error(errorMessage));

        await rosService.initialize();

        expect(rosService.initialized).to.be.false;
    });

    it('should shutdown ROS context', () => {
        rosService.shutdown();

        expect(shutdownStub.calledOnce).to.be.true;
        expect(rosService.initialized).to.be.false;
    });

    it('should return existing node if created', async () => {
        const nodeName = 'test_node';
        const existingNode = nodeStub;
        rosService.createdNodes.set(nodeName, existingNode);

        const node = rosService.getNode(nodeName);

        expect(node).to.exist;
        expect(node).to.be.an.instanceOf(rclnodejs.Node);
        expect(rosService.createdNodes.get(nodeName)).to.equal(existingNode);
    });

    it('should create and return new node if not created', async () => {
        const nodeConstStub = stub(rclnodejs, 'Node');
        nodeConstStub.returns(nodeStub);
    
        const newNode = rosService.getNode('new node'); 
    
        expect(newNode).to.exist;
    });
    
});
