import { CommandService } from "@src/services/command.service";
import Logger from "@src/utils/Logger";
import { Router } from "express";
import { StatusCodes } from "http-status-codes";
import { Service } from "typedi";
import { FileService } from "@src/services/file.service";

@Service()
export class CommandController {
    private readonly LOGGER: Logger;
    router: Router;
    
    constructor(
        private readonly commandService: CommandService, private readonly fileService: FileService
	) {
		this.configureRouter();
		this.LOGGER = new Logger('CommandController', fileService);
	}

    private configureRouter(): void {
        this.router = Router();

        /**
		 * @swagger
		 *
		 * /api/command/identify:
		 *     post:
		 *       description: Identify Robot
		 *       tags:
		 *         - Command
		 *       requestBody:
		 *           description: Identification
		 *           required: true
		 *           content:
		 *             application/json:
		 *               schema:
		 *                 type: object
		 *                 properties:
		 *                   robot_id:
		 *                     type: string
		 *                     description: Robot Hostname or all for all
		 *       responses:
		 *         200:
		 *           description: OK
         *         500:
         *           description: Internal Server Error
		 */
		this.router.post('/identify', async (req, res) => {
            try {
                await this.commandService.identify(req.body.robot_id);
                res.status(StatusCodes.OK).send();
            } catch(e) {
                this.LOGGER.err('ERROR :', e);
				res.status(StatusCodes.INTERNAL_SERVER_ERROR).send();
            }
		});

		/**
		 * @swagger
		 *
		 * /api/command/start_mission:
		 *     post:
		 *       description: Start exploration Robot
		 *       tags:
		 *         - Command
		 *       responses:
		 *         200:
		 *           description: OK
         *         500:
         *           description: Internal Server Error
		 */
		this.router.post('/start_mission', async (_, res) => {
            try {
                await this.commandService.startMission();
                res.status(StatusCodes.OK).send();
            } catch(e) {
                this.LOGGER.err('ERROR :', e);
				res.status(StatusCodes.INTERNAL_SERVER_ERROR).send();
            }
		});

		/**
		 * @swagger
		 *
		 * /api/command/stop_mission:
		 *     post:
		 *       description: Stop exploration Robot
		 *       tags:
		 *         - Command
		 *       responses:
		 *         200:
		 *           description: OK
         *         500:
         *           description: Internal Server Error
		 */
		this.router.post('/stop_mission', async (_, res) => {
            try {
                await this.commandService.stopMission();
                res.status(StatusCodes.OK).send();
            } catch(e) {
                this.LOGGER.err('ERROR :', e);
				res.status(StatusCodes.INTERNAL_SERVER_ERROR).send();
            }
		});
    }
}