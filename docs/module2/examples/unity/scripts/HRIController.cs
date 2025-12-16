using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class HRIController : MonoBehaviour
{
    [Header("Robot Control")]
    public GameObject robot;
    public float moveSpeed = 5.0f;
    public float turnSpeed = 100.0f;

    [Header("UI Elements")]
    public Slider speedSlider;
    public Button forwardButton;
    public Button backwardButton;
    public Button leftButton;
    public Button rightButton;
    public TextMeshProUGUI statusText;

    [Header("Communication")]
    public TextMeshProUGUI robotResponseText;
    public InputField commandInputField;

    private Rigidbody robotRigidbody;
    private Vector3 movementDirection = Vector3.zero;
    private bool isMoving = false;

    void Start()
    {
        if (robot != null)
        {
            robotRigidbody = robot.GetComponent<Rigidbody>();
        }

        SetupUIControls();
    }

    void SetupUIControls()
    {
        if (speedSlider != null)
        {
            speedSlider.onValueChanged.AddListener(OnSpeedChanged);
        }

        if (forwardButton != null)
        {
            forwardButton.onClick.AddListener(() => MoveRobot(Vector3.forward));
        }
        if (backwardButton != null)
        {
            backwardButton.onClick.AddListener(() => MoveRobot(Vector3.back));
        }
        if (leftButton != null)
        {
            leftButton.onClick.AddListener(() => TurnRobot(-1.0f));
        }
        if (rightButton != null)
        {
            rightButton.onClick.AddListener(() => TurnRobot(1.0f));
        }

        if (commandInputField != null)
        {
            commandInputField.onEndEdit.AddListener(ProcessCommand);
        }
    }

    void Update()
    {
        HandleMovement();
        UpdateStatus();
    }

    void MoveRobot(Vector3 direction)
    {
        if (robotRigidbody != null)
        {
            movementDirection = direction;
            isMoving = true;
        }
    }

    void TurnRobot(float direction)
    {
        if (robot != null)
        {
            robot.transform.Rotate(Vector3.up, turnSpeed * direction * Time.deltaTime);
        }
    }

    void HandleMovement()
    {
        if (isMoving && robotRigidbody != null)
        {
            robotRigidbody.MovePosition(robotRigidbody.position +
                robot.transform.TransformDirection(movementDirection) * moveSpeed * Time.deltaTime);
        }
    }

    void OnSpeedChanged(float newSpeed)
    {
        moveSpeed = newSpeed * 5.0f; // Scale factor
    }

    void ProcessCommand(string command)
    {
        if (robotResponseText != null)
        {
            robotResponseText.text = $"Received command: {command}";

            // Process specific commands
            switch (command.ToLower())
            {
                case "move forward":
                    MoveRobot(Vector3.forward);
                    break;
                case "turn left":
                    TurnRobot(-1.0f);
                    break;
                case "turn right":
                    TurnRobot(1.0f);
                    break;
                case "stop":
                    isMoving = false;
                    movementDirection = Vector3.zero;
                    break;
            }
        }
    }

    void UpdateStatus()
    {
        if (statusText != null && robot != null)
        {
            statusText.text = $"Robot Position: {robot.transform.position:F2}\n" +
                             $"Status: {(isMoving ? "Moving" : "Stopped")}";
        }
    }
}