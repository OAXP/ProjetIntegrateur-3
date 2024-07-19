import { FileService } from "@src/services/file.service";
import { InfoService } from "@src/services/info.service";
import Logger from "@src/utils/Logger";
import { Router } from "express";
import { Service } from "typedi";
import { StatusCodes } from "http-status-codes";

@Service()
export class InfoController {
    private readonly LOGGER: Logger;
    router: Router;

    constructor(private readonly fileService: FileService, private readonly infoService: InfoService) {
        this.configureRouter();
        this.LOGGER = new Logger('InfoController', fileService);
    }

    private configureRouter(): void {
        this.router = Router();

        /**
         * @swagger
         *
         * /api/info/init_positions:
         *     post:
         *       description: Set initial positions
         *       tags:
         *         - Info
         *       requestBody:
         *           description: Initial positions
         *           required: true
         *           content:
         *             application/json:
         *               schema:
         *                 type: object
         *                 properties:
         *                   robot_id:
         *                     type: object
         *                     description: Initial positions and orientations
         *                     properties:
         *                      init_position:
         *                       type: object
         *                       description: initial position
         *                       properties:
         *                        x:
         *                         type: number
         *                         description: x-coordinate
         *                        y:
         *                         type: number
         *                         description: y-coordinate
         *                      init_orientation:
         *                       type: object
         *                       description: initial orientation (quaternion)
         *                       properties:
         *                        x:
         *                         type: number
         *                         description: x-coordinate
         *                        y:
         *                         type: number
         *                         description: y-coordinate
         *                        z:
         *                         type: number
         *                         description: z-coordinate
         *                        w:
         *                         type: number
         *                         description: w-coordinate
         *       responses:
         *         200:
         *           description: OK
         *         500:
         *           description: Internal Server Error
         */
        this.router.post('/init_positions', async (req, res) => {
            try {
                this.infoService.setInitPositions(req.body);
                res.status(StatusCodes.OK).send();
            } catch (e) {
                this.LOGGER.err('ERROR :', e);
                res.status(StatusCodes.INTERNAL_SERVER_ERROR).send();
            }
        });
    }
}