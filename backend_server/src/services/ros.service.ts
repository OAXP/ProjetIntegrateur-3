import Logger from "@src/utils/Logger";
import * as rclnodejs from 'rclnodejs';
import { Service } from "typedi";
import { FileService } from "./file.service";

@Service()
export class RosService {
    private readonly LOGGER: Logger;
    initialized: boolean = false;
    createdNodes: Map<string, rclnodejs.Node> = new Map();

    constructor(private readonly fileService: FileService) {
        this.LOGGER = new Logger('RosService', fileService);
    }

    async initialize() {
        try {
            await rclnodejs.init();
            this.LOGGER.info('ROS context initialized');
            this.initialized = true;
        } catch(error) {
            this.LOGGER.err(error);
        }
        
    }

    shutdown() {
        rclnodejs.shutdown();
        this.initialized = false;
    }

    getNode(name: string): rclnodejs.Node {
        const node = this.createdNodes.get(name) ?? new rclnodejs.Node(name);
        if (!this.createdNodes.get(name)){
            this.createdNodes.set(name, node);
            node.spin();
        }
        
        return node;
    }
}